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

    # Helper to load mesh safely
    def load_mesh(name, filename, color, pos=(0, 0, 0)):
        # Check file existence
        if not os.path.exists(filename):
            print(f"Warning: {filename} not found. Using placeholder.")
            if "shaft" in name:
                return server.scene.add_cylinder(name, 0.005, 0.1, color=color, position=pos)
            else:
                return server.scene.add_box(
                    name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
                )

        # Load mesh
        try:
            mesh = trimesh.load_mesh(filename)
        except Exception as e:
            print(f"Failed to load {filename}: {e}")
            return server.scene.add_box(
                name, dimensions=(0.04, 0.04, 0.1), color=color, position=pos
            )

        # Apply scale and color
        mesh.apply_scale(0.001)
        mesh.visual = ColorVisuals(mesh=mesh, face_colors=to_rgba(color))

        # Add to Viser
        return server.scene.add_mesh_trimesh(name, mesh, position=pos)

    # Load P16 models
    load_mesh("p16_base", "p16_base.stl", color=(50, 50, 50), pos=(0, 0, 0))  # Dark Grey
    p16_shaft = load_mesh(
        "p16_shaft", "p16_shaft.stl", color=(200, 200, 200), pos=(0, 0, 0)  # Silver
    )

    # Define GUI Layout
    with server.gui.add_folder("P16 Linear Actuator"):

        # Telemetry display
        telemetry_md = server.gui.add_markdown("**Status:** Waiting for telemetry...")

        # Control buttons
        btn_extend = server.gui.add_button("Extend", color="green", icon=viser.Icon.ARROW_UP)
        btn_stop = server.gui.add_button("Brake", color="red", icon=viser.Icon.SQUARE)
        btn_retract = server.gui.add_button("Retract", color="yellow", icon=viser.Icon.ARROW_DOWN)

        # MOCK MODE CONTROLS
        # Only show this slider if we are NOT connected to hardware
        if ser is None:
            server.gui.add_markdown("---")  # Divider
            server.gui.add_markdown("**Mock Simulation:**")
            mock_slider = server.gui.add_slider(
                "Position (mm)", min=0.0, max=150.0, step=1.0, initial_value=0.0
            )

            @mock_slider.on_update
            def _(_):
                pos_mm = mock_slider.value

                # Update 3D Model
                extension_meters = pos_mm / 1000.0
                p16_shaft.position = (0.0, 0.0, extension_meters)

                # Update Telemetry Text to simulate firmware response
                telemetry_md.content = (
                    f"**Position:** {pos_mm:.2f} mm\n\n" f"**Raw ADC:** SIM\n\n" f"**Status:** MOCK"
                )

    # Helper to send commands
    def send_command(cmd_id, name):
        if ser:
            packet = create_packet(cmd_id)
            ser.write(packet)
        else:
            print(f"[MOCK] Sending {name} command ({hex(cmd_id)})")

    # Bind buttons
    @btn_extend.on_click
    def _(_):
        send_command(MSG_P16_EXTEND, "Extend")

    @btn_stop.on_click
    def _(_):
        send_command(MSG_P16_BRAKE, "Brake")

    @btn_retract.on_click
    def _(_):
        send_command(MSG_P16_RETRACT, "Retract")

    # Serial read loop
    def read_serial_loop():
        while True:
            if ser and ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("STATUS"):
                        # "STATUS <Pos_mm> <Raw_ADC> <Fault>"
                        parts = line.split()
                        if len(parts) >= 4:
                            pos_mm = float(parts[1])
                            raw_adc = parts[2]
                            fault_bool = parts[3]

                            fault_status = "FAULT" if fault_bool == "true" else "OK"
                            color = "red" if fault_status == "FAULT" else "green"

                            # Update Text
                            telemetry_md.content = (
                                f"**Position:** {pos_mm:.2f} mm\n\n"
                                f"**Raw ADC:** {raw_adc}\n\n"
                                f"**Status:** <span style='color:{color}'>{fault_status}</span>"
                            )

                            # Update 3D Model
                            extension_meters = pos_mm / 1000.0  # convert mm to meters
                            p16_shaft.position = (0.0, 0.0, extension_meters)

                    elif line:
                        print(f"[FW] {line}")
                except ValueError:
                    pass  # Handle float conversion errors
                except Exception as e:
                    print(f"Serial Error: {e}")

            time.sleep(0.01)

    # Start the reader thread
    t = threading.Thread(target=read_serial_loop, daemon=True)
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
