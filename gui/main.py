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

# Protocol Constants
START_BYTE = 0xA5
MSG_P16_EXTEND = 0x30
MSG_P16_RETRACT = 0x31
MSG_P16_BRAKE = 0x32
MSG_T16_EXTEND = 0x40
MSG_T16_RETRACT = 0x41
MSG_T16_BRAKE = 0x42
MSG_PING = 0x50

# Command lookup for debug printing
CMD_NAMES = {
    MSG_P16_EXTEND: "P16_EXTEND",
    MSG_P16_RETRACT: "P16_RETRACT",
    MSG_P16_BRAKE: "P16_BRAKE",
    MSG_T16_EXTEND: "T16_EXTEND",
    MSG_T16_RETRACT: "T16_RETRACT",
    MSG_T16_BRAKE: "T16_BRAKE",
    MSG_PING: "PING",
}

# Nordic UART Service (NUS) RX Characteristic UUID
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

# Global BLE State
ble_client = None
ble_loop = None


def create_packet(msg_id, payload=None):
    """
    Creates a binary packet.
    No payload (Ping, Brake): [START_BYTE, MSG_ID, CHECKSUM] — checksum = ID.
    With payload (Extend, Retract): [START_BYTE, MSG_ID, PAYLOAD, CHECKSUM] — checksum = (ID + PAYLOAD) & 0xFF.
    """
    if payload is None:
        return bytes([START_BYTE, msg_id, msg_id & 0xFF])
    payload = max(0, min(255, int(payload)))
    checksum = (msg_id + payload) & 0xFF
    return bytes([START_BYTE, msg_id, payload, checksum])


async def connect_ble():
    global ble_client
    print("[BLE] Scanning for OmniTile_1 (or NUS UUID)...")

    def match_device(device, adv):
        has_name = (device.name == "OmniTile_1") or (adv.local_name == "OmniTile_1")
        has_uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e" in adv.service_uuids
        return has_name or has_uuid

    target = await BleakScanner.find_device_by_filter(match_device, timeout=10.0)

    if target:
        print(f"[BLE] Found {target.name or 'Device'}! Connecting...")
        client = BleakClient(target.address)
        try:
            await client.connect()
            print("[BLE] Connected successfully!")
            ble_client = client
        except Exception as e:
            print(f"[BLE] Failed to connect: {e}")
    else:
        print("[BLE] OmniTile_1 not found. Ensure it is powered on and advertising.")


def start_ble_thread():
    """Starts a dedicated asyncio event loop for Bleak in a background thread."""
    global ble_loop
    ble_loop = asyncio.new_event_loop()

    def run_loop():
        asyncio.set_event_loop(ble_loop)
        if ble_loop is not None:
            ble_loop.run_forever()

    t = threading.Thread(target=run_loop, daemon=True)
    t.start()

    # Schedule the connection coroutine on the new loop
    asyncio.run_coroutine_threadsafe(connect_ble(), ble_loop)


def main():
    parser = argparse.ArgumentParser(description="OmniTiles Debug GUI")
    parser.add_argument("--port", type=str, default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    # 1. Start Serial (Fallback/Telemetry)
    ser = None
    if args.port:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
            print(f"[UART] Connected to {args.port} at {args.baud} baud")
        except serial.SerialException as e:
            print(f"[UART] Could not open serial port: {e}")
    else:
        print("[UART] No --port specified.")

    # 2. Start BLE (Primary Command Link)
    start_ble_thread()

    # 3. Start Viser Server
    server = viser.ViserServer(label="OmniTiles Debugger")
    server.scene.add_grid("ground", width=0.5, height=0.5, cell_size=0.05)

    def send_cmd(cmd_id, payload=None):
        global ble_client, ble_loop
        packet = create_packet(cmd_id, payload)
        cmd_name = CMD_NAMES.get(cmd_id, f"UNKNOWN_{cmd_id:02X}")
        payload_str = f" speed={payload}" if payload is not None else ""

        # Try BLE First
        if ble_client and ble_client.is_connected and ble_loop is not None:
            asyncio.run_coroutine_threadsafe(
                ble_client.write_gatt_char(NUS_RX_UUID, packet, response=False),
                ble_loop,
            )
            print(f"[BLE TX] {cmd_name}{payload_str}")
        # Fallback to UART
        elif ser:
            ser.write(packet)
            print(f"[UART TX] {cmd_name}{payload_str}")
        # Mock Mode
        else:
            print(f"[MOCK TX] {cmd_name}{payload_str}")

    # GUI: System Controls
    with server.gui.add_folder("System Controls"):
        server.gui.add_button("Send Ping", color="blue").on_click(lambda _: send_cmd(MSG_PING))

    # Shared state for speed (0–255); sliders show 10–100%
    state = {"speed_p16": 255, "speed_t16": 255}

    # GUI: P16 Controls
    with server.gui.add_folder("P16 Linear Actuator"):
        p16_md = server.gui.add_markdown("Waiting...")
        speed_p16 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_p16.on_update
        def _(_):
            state["speed_p16"] = int(speed_p16.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send_cmd(MSG_P16_EXTEND, state["speed_p16"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_P16_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_P16_RETRACT, state["speed_p16"])
        )

    # GUI: T16 Controls
    with server.gui.add_folder("T16 Track Actuator"):
        t16_md = server.gui.add_markdown("Waiting...")
        speed_t16 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_t16.on_update
        def _(_):
            state["speed_t16"] = int(speed_t16.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send_cmd(MSG_T16_EXTEND, state["speed_t16"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_T16_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_T16_RETRACT, state["speed_t16"])
        )

    def read_loop():
        while True:
            if ser and ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print(f"[STM32] {line}")
                except Exception as e:
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
