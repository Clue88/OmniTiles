# OmniTiles

OmniTiles is a robotics testing platform for ESE Senior Design (2025-26). The main controller is an
STM32F7 that drives the actuators connected to a DWM3001CDK that acts as a wireless bridge (BLE with
Nordic UART Service to STM32 over SPI). A Python GUI can be used for debugging and sending commands.

## Project Layout

**[omnitiles/](omnitiles/)** — Rust firmware for the STM32F7. Handles command parsing, motor
control, and communication over UART and SPI. This is the main firmware.

**[dwm_tag/](dwm_tag/)** — Zephyr app for the nRF52 DWM tag. Receives data over the Nordic UART
Service and passes it to the STM32 over SPI.

**[gui/](gui/)** — Python debug GUI (Viser). Web UI on localhost:8080 to send commands. Connects
over BLE to the DWM tag or optionally over serial.

## System Diagram

```
┌─────────────────┐     BLE (NUS)      ┌──────────────┐    SPI (slave)     ┌─────────────────┐
│  GUI            │ ─────────────────► │  DWM tag     │ ─────────────────► │  OmniTiles      │
│  (Python)       │                    │  (nRF52)     │   DRDY + payload   │  firmware       │
│  localhost:8080 │                    │  OmniTile_1  │                    │  (STM32F7)      │
└─────────────────┘                    └──────────────┘                    └─────────────────┘
        │                                                                          │
        │  UART (e.g. dev board or USB–UART bridge on PCB)                         │
        └──────────────────────────────────────────────────────────────────────────┘
```

Commands from host to STM32 all use the same format: sync byte `0xA5`, message ID, checksum. See
[omnitiles/src/protocol/messages.rs](omnitiles/src/protocol/messages.rs) for the IDs and
[omnitiles/src/protocol/parser.rs](omnitiles/src/protocol/parser.rs) for the parser.

## Quick Start

- **STM32 firmware:** [omnitiles/README.md](omnitiles/README.md) — Rust toolchain, flash, and how to
attach a terminal (`screen` at 115200).
- **DWM tag (nRF52):** [dwm_tag/README.md](dwm_tag/README.md) — building and flashing with the nRF
Connect extension in VS Code.
- **GUI:** [gui/README.md](gui/README.md) — run the Python app, BLE vs UART.

## Documentation

Rust API docs: from the repo root, `cd omnitiles && cargo doc --no-deps --open`. The same docs are
built in CI and published to GitHub Pages on push to `main`. Protocol message definitions and the
parser are in the `omnitiles/src/protocol/` tree.

## License

MIT. See [LICENSE](LICENSE).
