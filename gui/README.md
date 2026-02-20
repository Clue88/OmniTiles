# OmniTiles debug GUI

Python GUI for driving the tile during development. It runs a Viser server (web UI at http://localhost:8080) with buttons for Ping and for the P16 linear and T16 track actuators (extend, retract, brake). You can also plug in a serial port to see whatever the STM32 prints.

The GUI prefers BLE: it scans for a device advertising as **OmniTile_1** (the [DWM tag](../dwm_tag/)) and sends command packets over the Nordic UART Service. If you pass `--port`, it also opens that serial port; when BLE isn’t connected it will send commands over UART instead, and it always reads lines from UART and prints them to the console so you can see STM32 debug output. So you can use BLE only (PCB + DWM tag), UART only (dev board), or both. Full picture: [root README](../README.md).

**Protocol:** Same as the firmware. Each packet is three bytes: sync `0xA5`, message ID (e.g. PING `0x50`, P16/T16 extend/retract/brake), and a checksum byte (for these packets, equal to the message ID). The IDs are in [omnitiles/src/protocol/messages.rs](../omnitiles/src/protocol/messages.rs); the GUI mirrors them in `main.py`.

**Requirements:** Python 3.12+. Dependencies are in [pyproject.toml](pyproject.toml) (bleak, pyserial, trimesh, viser).

**Run:**

```bash
uv sync
uv run python main.py
```

Or with a venv: `pip install -e .` then `python main.py`.

Optional: `--port` for the serial device (e.g. `--port /dev/tty.usbmodem1103`) and `--baud` (default 115200). Then open http://localhost:8080; the System Controls folder has Ping, and the P16 / T16 folders have the actuator buttons. If BLE is up, commands go over BLE; otherwise over UART when `--port` is set, or they’re just printed as MOCK.
