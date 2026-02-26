import argparse
import asyncio
import math
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
    "buffer_bottom_mm": 20.0,
    "buffer_top_mm": 35.0,
    "base_stl": "p16_base.stl",
    "move_stl": "p16_shaft.stl",
}

M2_CONFIG = {
    "name": "T16 Track Actuator",
    "stroke_mm": 100.0,
    "buffer_bottom_mm": 25.0,
    "buffer_top_mm": 15.0,
    "base_stl": "t16_base.stl",
    "move_stl": "t16_carriage.stl",
}

# UWB anchor positions (X, Y) in meters. Anchor 1 origin, 2 on X-axis, 3 on Y-axis.
ANCHORS_M = {
    1: (0.0, 0.0),
    2: (3.0, 0.0),
    3: (0.0, 3.0),
}


def _min_position_mm(config):
    return config["buffer_bottom_mm"]


def _max_position_mm(config):
    return config["stroke_mm"] - config["buffer_top_mm"]


def trilaterate_2d(r1, r2, r3):
    """2D trilateration from three distances (m) to anchors 1, 2, 3. Returns (X, Y) in meters."""
    a = ANCHORS_M[2][0]
    b = ANCHORS_M[3][1]
    x = (r1**2 - r2**2 + a**2) / (2.0 * a)
    y = (r1**2 - r3**2 + b**2) / (2.0 * b)
    return (x, y)


# --- Protocol Constants ---
START_BYTE = 0xA5
MSG_M1_EXTEND = 0x30
MSG_M1_RETRACT = 0x31
MSG_M1_BRAKE = 0x32
MSG_M1_SET_POSITION = 0x33
MSG_M2_EXTEND = 0x40
MSG_M2_RETRACT = 0x41
MSG_M2_BRAKE = 0x42
MSG_M2_SET_POSITION = 0x43
MSG_PING = 0x50
MSG_TELEMETRY = 0x60

CMD_NAMES = {
    MSG_M1_EXTEND: f"{M1_CONFIG['name']}_EXTEND",
    MSG_M1_RETRACT: f"{M1_CONFIG['name']}_RETRACT",
    MSG_M1_BRAKE: f"{M1_CONFIG['name']}_BRAKE",
    MSG_M1_SET_POSITION: f"{M1_CONFIG['name']}_SET_POSITION",
    MSG_M2_EXTEND: f"{M2_CONFIG['name']}_EXTEND",
    MSG_M2_RETRACT: f"{M2_CONFIG['name']}_RETRACT",
    MSG_M2_BRAKE: f"{M2_CONFIG['name']}_BRAKE",
    MSG_M2_SET_POSITION: f"{M2_CONFIG['name']}_SET_POSITION",
    MSG_PING: "PING",
}

# Nordic UART Service UUIDs
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# Global State
ble_client = None
ble_loop = None


def create_packet(msg_id, payload=None):
    """Creates a binary packet. 3 bytes if no payload, 4 bytes if payload."""
    if payload is not None:
        payload = max(0, min(255, int(payload)))
        checksum = (msg_id + payload) & 0xFF
        return bytes([START_BYTE, msg_id, payload, checksum])
    checksum = msg_id & 0xFF
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

    def load_mesh(name, filename, color, pos, rotation=None):
        if not os.path.exists(filename):
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )
        try:
            mesh = trimesh.load_mesh(filename)
            mesh.apply_scale(0.001)
            mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))
            if rotation is not None:
                mesh.apply_transform(rotation)
            return server.scene.add_mesh_trimesh(name, mesh, position=pos)
        except Exception as e:
            print(f"Load error {name}: {e}")
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )

    # Load M1 Models
    m1_is_t16 = "t16" in M1_CONFIG["move_stl"].lower()
    m1_carriage_offset_z = -0.013 if m1_is_t16 else 0.0

    # Primary M1
    load_mesh("m1_base", M1_CONFIG["base_stl"], color=(50, 50, 50), pos=(0.0, 0.0, 0.0))
    m1_shaft = load_mesh(
        "m1_shaft",
        M1_CONFIG["move_stl"],
        color=(200, 200, 200),
        pos=(0.0, 0.0, m1_carriage_offset_z),
    )

    # Secondary M1, mirrored in height (150 - x) and offset in X
    load_mesh("m1b_base", M1_CONFIG["base_stl"], color=(50, 50, 50), pos=(-0.06, 0.0, 0.0))
    m1b_shaft = load_mesh(
        "m1b_shaft",
        M1_CONFIG["move_stl"],
        color=(200, 200, 200),
        pos=(-0.06, 0.0, m1_carriage_offset_z),
    )

    # Load M2 Models (horizontal orientation along Y)
    m2_is_t16 = "t16" in M2_CONFIG["move_stl"].lower()
    # For M2 T16, apply the 13 mm adjustment as a constant offset along the horizontal travel axis
    m2_carriage_offset_y = -0.013 if m2_is_t16 else 0.0
    m2_carriage_offset_z = 0.0
    m2_rotation = trimesh.transformations.rotation_matrix(math.radians(-90.0), [1.0, 0.0, 0.0])

    # First M2
    load_mesh(
        "m2_base",
        M2_CONFIG["base_stl"],
        color=(50, 50, 80),
        pos=(0.1, 0.0, 0.0),
        rotation=m2_rotation,
    )
    m2_carriage = load_mesh(
        "m2_carriage",
        M2_CONFIG["move_stl"],
        color=(200, 200, 200),
        pos=(0.1, m2_carriage_offset_y, m2_carriage_offset_z),
        rotation=m2_rotation,
    )

    # Second M2, mirrored in length
    load_mesh(
        "m2b_base",
        M2_CONFIG["base_stl"],
        color=(50, 50, 80),
        pos=(0.16, 0.0, 0.0),
        rotation=m2_rotation,
    )
    m2b_carriage = load_mesh(
        "m2b_carriage",
        M2_CONFIG["move_stl"],
        color=(200, 200, 200),
        pos=(0.16, m2_carriage_offset_y, m2_carriage_offset_z),
        rotation=m2_rotation,
    )

    # Initialize actuators to known positions with no telemetry
    m1_mm_init = 0.0
    m1_z_init = (m1_mm_init - 13) / 1000.0 if m1_is_t16 else m1_mm_init / 1000.0
    m1_shaft.position = (0.0, 0.0, m1_z_init)

    m1b_mm_init = M1_CONFIG["stroke_mm"] - m1_mm_init
    m1b_z_init = (m1b_mm_init - 13) / 1000.0 if m1_is_t16 else m1b_mm_init / 1000.0
    m1b_shaft.position = (-0.06, 0.0, m1b_z_init)

    m2_mm_init = 0.0
    m2_y_init = m2_mm_init / 1000.0
    m2_carriage.position = (0.1, m2_carriage_offset_y + m2_y_init, m2_carriage_offset_z)
    m2b_carriage.position = (0.16, m2_carriage_offset_y + m2_y_init, m2_carriage_offset_z)

    # 3. Telemetry Callback Logic
    def handle_telemetry(sender, data: bytearray):
        if len(data) >= 11 and data[0] == START_BYTE and data[1] == MSG_TELEMETRY:
            m1_pos_adc = data[2]
            m2_pos_adc = data[3]
            uwb1_mm = (data[4] << 8) | data[5]
            uwb2_mm = (data[6] << 8) | data[7]
            uwb3_mm = (data[8] << 8) | data[9]
            checksum = sum(data[1:10]) & 0xFF

            if data[10] == checksum:
                m1_mm = (m1_pos_adc / 255.0) * M1_CONFIG["stroke_mm"]
                m2_mm = (m2_pos_adc / 255.0) * M2_CONFIG["stroke_mm"]

                uwb1_m = uwb1_mm / 1000.0
                uwb2_m = uwb2_mm / 1000.0
                uwb3_m = uwb3_mm / 1000.0
                pos_x, pos_y = trilaterate_2d(uwb1_m, uwb2_m, uwb3_m)

                # Update markdown with ADC, position, and UWB
                m1_md.content = f"**ADC:** {m1_pos_adc} | **Est. Pos:** {m1_mm:.1f} mm | **UWB (X, Y):** ({pos_x:.3f}, {pos_y:.3f}) m"
                m2_md.content = f"**ADC:** {m2_pos_adc} | **Est. Pos:** {m2_mm:.1f} mm | **UWB (X, Y):** ({pos_x:.3f}, {pos_y:.3f}) m"

                # Update 3D Models
                m1_z = (m1_mm - 13) / 1000.0 if m1_is_t16 else m1_mm / 1000.0
                # Mirrored M1: 150 - x mm
                m1b_mm = M1_CONFIG["stroke_mm"] - m1_mm
                m1b_z = (m1b_mm - 13) / 1000.0 if m1_is_t16 else m1b_mm / 1000.0
                m2_y = m2_mm / 1000.0
                m1_shaft.position = (0.0, 0.0, m1_z)
                m1b_shaft.position = (-0.06, 0.0, m1b_z)
                m2_carriage.position = (0.1, m2_carriage_offset_y + m2_y, m2_carriage_offset_z)
                m2b_carriage.position = (0.16, m2_carriage_offset_y + m2_y, m2_carriage_offset_z)

    # 4. Start BLE
    start_ble_thread(handle_telemetry)

    # 5. GUI Commands
    def send_cmd(cmd_id, payload=None):
        global ble_client, ble_loop
        packet = create_packet(cmd_id, payload)
        cmd_name = CMD_NAMES.get(cmd_id, f"UNKNOWN_{cmd_id:02X}")
        payload_str = f" speed={payload}" if payload is not None else ""

        if ble_client and ble_client.is_connected and ble_loop is not None:
            asyncio.run_coroutine_threadsafe(
                ble_client.write_gatt_char(NUS_RX_UUID, packet, response=False), ble_loop
            )
            print(f"[BLE TX] {cmd_name}{payload_str}")
        elif ser:
            ser.write(packet)
            print(f"[UART TX] {cmd_name}{payload_str}")
        else:
            print(f"[MOCK TX] {cmd_name}{payload_str}")

    # Shared state for PWM speed (0–255); sliders show 10–100%
    state = {"speed_m1": 255, "speed_m2": 255}

    with server.gui.add_folder("System Controls"):
        server.gui.add_button("Send Ping", color="blue").on_click(lambda _: send_cmd(MSG_PING))

    with server.gui.add_folder(f"M1: {M1_CONFIG['name']}"):
        m1_md = server.gui.add_markdown("Waiting...")
        speed_m1 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_m1.on_update
        def _(_):
            state["speed_m1"] = int(speed_m1.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send_cmd(MSG_M1_EXTEND, state["speed_m1"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_M1_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_M1_RETRACT, state["speed_m1"])
        )
        m1_slider = server.gui.add_slider(
            "Target Position (mm)",
            min=int(_min_position_mm(M1_CONFIG)),
            max=int(_max_position_mm(M1_CONFIG)),
            step=1,
            initial_value=int(_min_position_mm(M1_CONFIG)),
        )
        m1_slider.on_update(lambda event: send_cmd(MSG_M1_SET_POSITION, int(event.target.value)))

    with server.gui.add_folder(f"M2: {M2_CONFIG['name']}"):
        m2_md = server.gui.add_markdown("Waiting...")
        speed_m2 = server.gui.add_slider("Speed %", min=10, max=100, step=1, initial_value=100)

        @speed_m2.on_update
        def _(_):
            state["speed_m2"] = int(speed_m2.value * 2.55)

        server.gui.add_button("Extend", color="green").on_click(
            lambda _: send_cmd(MSG_M2_EXTEND, state["speed_m2"])
        )
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_cmd(MSG_M2_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_cmd(MSG_M2_RETRACT, state["speed_m2"])
        )
        m2_slider = server.gui.add_slider(
            "Target Position (mm)",
            min=int(_min_position_mm(M2_CONFIG)),
            max=int(_max_position_mm(M2_CONFIG)),
            step=1,
            initial_value=int(_min_position_mm(M2_CONFIG)),
        )
        m2_slider.on_update(lambda event: send_cmd(MSG_M2_SET_POSITION, int(event.target.value)))

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
