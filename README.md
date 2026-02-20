# OmniTiles

**OmniTiles** is a robotics platform developed for ESE Senior Design (Fall 2025). It consists of an STM32F7-based main controller that drives actuators (e.g., linear lift and track/tilt), a wireless BLE-to-SPI bridge (nRF52) for phone/PC control, and a Python debug GUI for development and testing.

## Project layout

| Directory | Role |
| --------- | ----- |
| [**omnitiles/**](omnitiles/) | Embedded firmware (Rust) for the main MCU (STM32F767). Parses commands, drives motors, and exposes UART/SPI for host communication. |
| [**dwm_tag/**](dwm_tag/) | Zephyr application for an nRF52 “DWM tag”: BLE peripheral that forwards commands to the STM32 over SPI. Used on the OmniTiles PCB. |
| [**gui/**](gui/) | Python debug GUI (Viser + BLE/UART) to send commands to the tile and view telemetry. |

## How the pieces connect

```
┌─────────────────┐     BLE (NUS)      ┌──────────────┐    SPI (slave)     ┌─────────────────┐
│  GUI            │ ─────────────────► │  DWM tag     │ ─────────────────► │  OmniTiles      │
│  (Python)       │                    │  (nRF52)     │   DRDY + payload   │  firmware       │
│  localhost:8080 │                    │  OmniTile_1  │                    │  (STM32F7)      │
└─────────────────┘                    └──────────────┘                    └─────────────────┘
        │                                                                          │
        │  UART (optional, e.g. dev board or USB-UART bridge on PCB)               │
        └──────────────────────────────────────────────────────────────────────────┘
```

- **On the PCB (v1):** The GUI talks to the nRF52 over BLE. The nRF52 receives packets, queues them, and when the STM32 asserts SPI chip-select and reads, the tag drives **DRDY** high, provides the payload over SPI, then clears DRDY. The STM32 firmware reads from SPI when DRDY is high and also from UART (e.g. debug).
- **On a dev board:** You can skip the DWM tag and connect the GUI (or a terminal) to the STM32 via UART at 115200 baud.

All host-to-STM32 commands use the same [command protocol](omnitiles/src/protocol/messages.rs): sync byte `0xA5`, message ID, checksum. The firmware parser lives in `omnitiles/src/protocol/parser.rs`.

## Quick start

1. **Firmware (STM32)**  
   See [omnitiles/README.md](omnitiles/README.md) for Rust toolchain, flashing, and debug UART (`screen` at 115200).

2. **BLE bridge (nRF52, for PCB)**  
   See [dwm_tag/README.md](dwm_tag/README.md) for building and flashing the DWM tag.

3. **Debug GUI**  
   See [gui/README.md](gui/README.md) for running the Python GUI (BLE + optional UART).

## Documentation

- **Rust (omnitiles):** From the repo root, run `cargo doc --no-deps --open` inside `omnitiles/`. CI also builds and deploys these docs to GitHub Pages on pushes to `main`.
- **Protocol:** Message IDs and command types are in `omnitiles/src/protocol/messages.rs`; the parser is in `omnitiles/src/protocol/parser.rs`.

## License

MIT. See [LICENSE](LICENSE) in the repository root.
