import argparse
import time
import serial
import threading
import viser

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

    # Define GUI Layout
    with server.gui.add_folder("P16 Linear Actuator"):

        # Telemetry display
        telemetry_md = server.gui.add_markdown("**Status:** Waiting for telemetry...")

        # Control buttons
        btn_extend = server.gui.add_button("Extend", color="green", icon=viser.Icon.ARROW_UP)
        btn_stop = server.gui.add_button("Brake", color="red", icon=viser.Icon.SQUARE)
        btn_retract = server.gui.add_button("Retract", color="yellow", icon=viser.Icon.ARROW_DOWN)

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
                    # Read line and decode
                    line = ser.readline().decode("utf-8", errors="ignore").strip()

                    if line.startswith("STATUS"):
                        # Firmware sends: "STATUS <Pos_mm> <Raw_ADC> <Fault>"
                        parts = line.split()
                        if len(parts) >= 4:
                            pos_mm = parts[1]
                            raw_adc = parts[2]
                            fault_bool = parts[3]

                            fault_status = "FAULT" if fault_bool == "true" else "OK"
                            color = "red" if fault_status == "FAULT" else "green"

                            # Update Viser Markdown
                            telemetry_md.content = (
                                f"### Telemetry\n"
                                f"**Position:** {pos_mm} mm\n\n"
                                f"**Raw ADC:** {raw_adc}\n\n"
                                f"**Status:** <span style='color:{color}'>{fault_status}</span>"
                            )
                    elif line:
                        print(f"[FW] {line}")

                except Exception as e:
                    print(f"Serial Read Error: {e}")

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
