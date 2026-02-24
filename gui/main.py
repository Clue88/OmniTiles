import argparse
import asyncio
import os
import threading
import time

import serial
import trimesh
import viser
from bleak import BleakClient, BleakScanner
from trimesh.visual import to_rgba
from trimesh.visual.color import ColorVisuals

# --- Hardware Configuration ---
M1_CONFIG = {
    "name": "P16 Linear Actuator",
    "stroke_mm": 150.0,
    "base_stl": "p16_base.stl",
    "move_stl": "p16_shaft.stl",
}

M2_CONFIG = {
    "name": "T16 Track Actuator",
    "stroke_mm": 100.0,
    "base_stl": "t16_base.stl",
    "move_stl": "t16_carriage.stl",
}

# --- Protocol Constants ---
START_BYTE = 0xA5
MSG_M1_EXTEND = 0x30
MSG_M1_RETRACT = 0x31
MSG_M1_BRAKE = 0x32
MSG_M2_EXTEND = 0x40
MSG_M2_RETRACT = 0x41
MSG_M2_BRAKE = 0x42
MSG_PING = 0x50
MSG_TELEMETRY = 0x60

CMD_NAMES = {
    MSG_M1_EXTEND: f"{M1_CONFIG['name']}_EXTEND",
    MSG_M1_RETRACT: f"{M1_CONFIG['name']}_RETRACT",
    MSG_M1_BRAKE: f"{M1_CONFIG['name']}_BRAKE",
    MSG_M2_EXTEND: f"{M2_CONFIG['name']}_EXTEND",
    MSG_M2_RETRACT: f"{M2_CONFIG['name']}_RETRACT",
    MSG_M2_BRAKE: f"{M2_CONFIG['name']}_BRAKE",
    MSG_PING: "PING",
}

# Nordic UART Service UUIDs
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Global State
ble_client = None
ble_loop = None


def create_packet(msg_id):
    """Creates a binary packet: [START_BYTE, MSG_ID, CHECKSUM]"""
    checksum = msg_id
    return bytes([START_BYTE, msg_id, checksum])


async def connect_ble(telemetry_callback):
    global ble_client

    def match_device(device, adv):
        has_name = (device.name == "OmniTile_1") or (adv.local_name == "OmniTile_1")
        has_uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e" in adv.service_uuids
        return has_name or has_uuid

    while True:
        if ble_client is None or not ble_client.is_connected:
            print("[BLE] Scanning for OmniTile_1 (or NUS UUID)...")
            target = await BleakScanner.find_device_by_filter(match_device, timeout=5.0)

            if target:
                print(f"[BLE] Found {target.name or 'Device'}! Connecting...")
                client = BleakClient(target.address)
                try:
                    await client.connect()
                    print("[BLE] Connected successfully!")
                    await client.start_notify(NUS_TX_UUID, telemetry_callback)
                    ble_client = client
                except Exception as e:
                    print(f"[BLE] Failed to connect: {e}")
        await asyncio.sleep(2.0)


def start_ble_thread(telemetry_callback):
    """Starts a dedicated asyncio event loop for Bleak in a background thread."""
    global ble_loop
    ble_loop = asyncio.new_event_loop()

    def run_loop():
        if ble_loop is not None:
            asyncio.set_event_loop(ble_loop)
            ble_loop.run_forever()

    t = threading.Thread(target=run_loop, daemon=True)
    t.start()

    asyncio.run_coroutine_threadsafe(connect_ble(telemetry_callback), ble_loop)


def main():
    parser = argparse.ArgumentParser(description="OmniTiles Debug GUI")
    parser.add_argument("--port", type=str, default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # 1. Start Serial
    ser = None
    if args.port:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
            print(f"[UART] Connected to {args.port} at {args.baud} baud")
        except serial.SerialException as e:
            print(f"[UART] Could not open serial port: {e}")

    # 2. Viser Server Setup
    server = viser.ViserServer(label="OmniTiles Debugger")
    server.scene.add_grid("ground", width=0.5, height=0.5, cell_size=0.05)

    def load_mesh(name, filename, color, pos):
        if not os.path.exists(filename):
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )
        try:
            mesh = trimesh.load_mesh(filename)
            mesh.apply_scale(0.001)
            mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))
            return server.scene.add_mesh_trimesh(name, mesh, position=pos)
        except Exception as e:
            print(f"Load error {name}: {e}")
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )

    # Load M1 Models
    load_mesh("m1_base", M1_CONFIG["base_stl"], color=(50, 50, 50), pos=(0, 0, 0))
    m1_shaft = load_mesh("m1_shaft", M1_CONFIG["move_stl"], color=(200, 200, 200), pos=(0, 0, 0))

    # Load M2 Models (Carriage offset by 13 mm)
    load_mesh("m2_base", M2_CONFIG["base_stl"], color=(50, 50, 80), pos=(0.1, 0, 0))
    m2_carriage = load_mesh(
        "m2_carriage", M2_CONFIG["move_stl"], color=(200, 200, 200), pos=(0.1, 0, -0.013)
    )

    # 3. Telemetry Callback Logic
    def handle_telemetry(sender, data: bytearray):
        if len(data) >= 5 and data[0] == START_BYTE and data[1] == MSG_TELEMETRY:
            m1_pos_adc = data[2]
            m2_pos_adc = data[3]
            checksum = (data[1] + data[2] + data[3]) & 0xFF

            if data[4] == checksum:
                # Convert 8-bit ADC (0-255) back into physical mm
                m1_mm = (m1_pos_adc / 255.0) * M1_CONFIG["stroke_mm"]
                m2_mm = (m2_pos_adc / 255.0) * M2_CONFIG["stroke_mm"]

                # Update Text
                m1_md.content = f"**ADC:** {m1_pos_adc} | **Est. Pos:** {m1_mm:.1f} mm"
                m2_md.content = f"**ADC:** {m2_pos_adc} | **Est. Pos:** {m2_mm:.1f} mm"

                # Update 3D Models
                m1_shaft.position = (0.0, 0.0, m1_mm / 1000.0)
                m2_carriage.position = (0.1, 0.0, (m2_mm - 13) / 1000.0)

    # 4. Start BLE
    start_ble_thread(handle_telemetry)

    # 5. GUI Commands
    def send_cmd(cmd_id):
        global ble_client, ble_loop
        packet = create_packet(cmd_id)
        cmd_name = CMD_NAMES.get(cmd_id, f"UNKNOWN_{cmd_id:02X}")

        if ble_client and ble_client.is_connected and ble_loop is not None:
            asyncio.run_coroutine_threadsafe(
                ble_client.write_gatt_char(NUS_RX_UUID, packet, response=False), ble_loop
            )
            print(f"[BLE TX] {cmd_name}")
        elif ser:
            ser.write(packet)
            print(f"[UART TX] {cmd_name}")
        else:
            print(f"[MOCK TX] {cmd_name}")

    with server.gui.add_folder("System Controls"):
        server.gui.add_button("Send Ping", color="blue").on_click(lambda _: send_cmd(MSG_PING))

    with server.gui.add_folder(f"M1: {M1_CONFIG['name']}"):
        m1_md = server.gui.add_markdown("Waiting...")
        server.gui.add_button("Extend", color="green").on_click(lambda _: send_cmd(MSG_M1_EXTEND))
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_M1_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_M1_RETRACT)
        )

    with server.gui.add_folder(f"M2: {M2_CONFIG['name']}"):
        m2_md = server.gui.add_markdown("Waiting...")
        server.gui.add_button("Extend", color="green").on_click(lambda _: send_cmd(MSG_M2_EXTEND))
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_M2_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_M2_RETRACT)
        )

    def read_loop():
        while True:
            if ser and ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print(f"[STM32] {line}")
                except Exception:
                    pass
            time.sleep(0.01)

    t = threading.Thread(target=read_loop, daemon=True)
    t.start()

    print("GUI Ready at http://localhost:8080")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")


if __name__ == "__main__":
    main()
