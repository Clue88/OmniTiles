import argparse
import os
import threading
import time

import serial
import trimesh
import viser
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


def create_packet(msg_id):
    """
    Creates a binary packet: [START_BYTE, MSG_ID, CHECKSUM]
    Checksum is the sum of ID bytes.
    """
    checksum = msg_id
    return bytes([START_BYTE, msg_id, checksum])


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="OmniTiles Debug GUI")
    parser.add_argument(
        "--port", type=str, default=None, help="Serial port (e.g., /dev/tty.usbmodem1234)"
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    # Connect to Serial
    ser = None
    if args.port:
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
            print(f"Connected to {args.port} at {args.baud} baud")
        except serial.SerialException as e:
            print(f"Could not open serial port: {e}")
            print("Running in MOCK mode (buttons will just print to console)")
    else:
        print("No --port specified. Running in MOCK mode.")

    # Start Viser Server
    server = viser.ViserServer(label="OmniTiles Debugger")

    # Add grid for context
    server.scene.add_grid("Ground Grid", width=0.5, height=0.5, cell_size=0.05)

    def load_mesh(name, filename, color, pos):
        if not os.path.exists(filename):
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )
        mesh = trimesh.load_mesh(filename)
        mesh.apply_scale(0.001)
        mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))
        return server.scene.add_mesh_trimesh(name, mesh, position=pos)

    # Load P16 models
    load_mesh("p16_base", "p16_base.stl", color=(50, 50, 50), pos=(0, 0, 0))
    p16_shaft = load_mesh("p16_shaft", "p16_shaft.stl", color=(200, 200, 200), pos=(0, 0, 0))

    # Load T16 models (carriage offset by 13 mm)
    load_mesh("t16_base", "t16_base.stl", (50, 50, 80), (0.1, 0, 0))
    t16_carriage = load_mesh("t16_carriage", "t16_carriage.stl", (200, 200, 200), (0.1, 0, -0.013))

    # Telemetry helpers
    def update_p16_telemetry(pos_mm, raw_adc, fault, md_handle):
        p16_shaft.position = (0.0, 0.0, pos_mm / 1000.0)
        status = "FAULT" if fault else "OK"
        md_handle.content = f"**Pos:** {pos_mm:.2f} mm | **ADC:** {raw_adc} | {status}"

    def update_t16_telemetry(pos_mm, raw_adc, fault, md_handle):
        t16_carriage.position = (0.1, 0.0, (pos_mm - 13) / 1000.0)  # Carriage offset 13 mm
        status = "FAULT" if fault else "OK"
        md_handle.content = f"**Pos:** {pos_mm:.2f} mm | **ADC:** {raw_adc} | {status}"

    def send_msg(msg_id):
        if ser:
            ser.write(create_packet(msg_id))

    # P16 Controls
    with server.gui.add_folder("P16 Linear Actuator"):
        p16_md = server.gui.add_markdown("Waiting...")
        server.gui.add_button("Extend", color="green").on_click(lambda _: send_msg(MSG_P16_EXTEND))
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_msg(MSG_P16_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_msg(MSG_P16_RETRACT)
        )

    # T16 Controls
    with server.gui.add_folder("T16 Track Actuator"):
        t16_md = server.gui.add_markdown("Waiting...")
        server.gui.add_button("Extend", color="green").on_click(lambda _: send_msg(MSG_T16_EXTEND))
        server.gui.add_button("Brake", color="red").on_click(lambda _: send_msg(MSG_T16_BRAKE))
        server.gui.add_button("Retract", color="yellow").on_click(
            lambda _: send_msg(MSG_T16_RETRACT)
        )

    # Mock Controls
    if ser is None:
        with server.gui.add_folder("Mock Simulation"):
            server.gui.add_markdown("Hardware disconnected. Use sliders to sim.")

            p16_slider = server.gui.add_slider("P16 (mm)", 0, 150, 1, 0)

            @p16_slider.on_update
            def _(_):
                update_p16_telemetry(p16_slider.value, "SIM", False, p16_md)

            t16_slider = server.gui.add_slider("T16 (mm)", 0, 100, 1, 0)

            @t16_slider.on_update
            def _(_):
                update_t16_telemetry(t16_slider.value, "SIM", False, t16_md)

    def read_loop():
        while True:
            if ser and ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    parts = line.split()

                    if line.startswith("STATUS_P16") and len(parts) >= 4:
                        update_p16_telemetry(float(parts[1]), parts[2], parts[3] == "true", p16_md)

                    elif line.startswith("STATUS_T16") and len(parts) >= 4:
                        update_t16_telemetry(float(parts[1]), parts[2], parts[3] == "true", t16_md)

                    elif line:
                        print(f"[FW] {line}")
                except Exception as e:
                    print(f"Rx Error: {e}")
            time.sleep(0.01)

    # Start the reader thread
    t = threading.Thread(target=read_loop, daemon=True)
    t.start()

    # Keep the main thread alive to serve the GUI
    print("GUI Ready at http://localhost:8080")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")


if __name__ == "__main__":
    main()
