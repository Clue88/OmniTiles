# OmniTiles debug GUI

Python debug GUI for the OmniTiles platform. It provides a **Viser** web interface (localhost:8080) with buttons to send commands to the tile (Ping, P16 linear actuator, T16 track actuator) and optionally shows telemetry from the STM32 over UART.

## Role in the project

The GUI is the main host-side tool for development and testing. It talks to the firmware either by:

- **BLE (primary):** Connects to the [DWM tag](../dwm_tag/) (advertised as **OmniTile_1**) and sends packets over the Nordic UART Service (NUS). The DWM tag forwards them to the STM32 via SPI.
- **UART (fallback):** If you pass `--port`, it also opens a serial connection (e.g. USB–UART on a dev board or via the RP2040 bridge on PCB v1). Commands can be sent over UART when BLE is not connected; UART is also used to print any lines the STM32 sends (e.g. debug messages).

So: **GUI → BLE or UART → (DWM tag → SPI or direct UART) → STM32 firmware.** For an overview, see the [repository root README](../README.md).

## Protocol

The GUI uses the same binary protocol as the firmware. Each packet is 3 bytes:

| Byte | Meaning |
|------|--------|
| 0 | Sync: `0xA5` |
| 1 | Message ID (e.g. PING `0x50`, P16/T16 extend/retract/brake) |
| 2 | Checksum (equal to message ID for these simple packets) |

Message IDs are defined in the [firmware protocol](../omnitiles/src/protocol/messages.rs); the GUI keeps matching constants in `main.py`.

## Requirements

- Python ≥ 3.12
- Dependencies are in [pyproject.toml](pyproject.toml): `bleak`, `pyserial`, `trimesh`, `viser`.

## Setup and run

From this directory (or repo root with `gui` as a package):

```bash
uv sync
uv run python main.py
```

Or with a virtualenv:

```bash
pip install -e .
python main.py
```

Optional arguments:

- `--port PORT` — Serial port for UART (e.g. `COM3` or `/dev/tty.usbmodem1103`).
- `--baud RATE` — Baud rate (default 115200).

Example with UART and default baud:

```bash
uv run python main.py --port /dev/tty.usbmodem1103
```

Then open **http://localhost:8080** in a browser. Use “System Controls” for Ping, and the P16 / T16 folders for the linear and track actuator buttons. If BLE is connected, commands are sent over BLE; otherwise they are sent over UART when `--port` is set, or printed as “MOCK” otherwise.

## Summary

| What | Detail |
|------|--------|
| **UI** | Viser server at http://localhost:8080 |
| **Command path** | BLE (OmniTile_1 / NUS) preferred; UART fallback if `--port` given |
| **Telemetry** | UART read loop prints STM32 lines to the console |
| **Protocol** | Same as [omnitiles protocol](../omnitiles/src/protocol/messages.rs) |
