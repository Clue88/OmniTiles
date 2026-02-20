# OmniTiles

OmniTiles is a robotics platform for ESE Senior Design (Fall 2025). The main controller is an STM32F7 that drives the actuators—linear lift, track/tilt, and so on. There’s an nRF52 “DWM tag” on the PCB that acts as a wireless bridge: it exposes BLE (Nordic UART Service) so a phone or PC can talk to it, and it forwards those bytes to the STM32 over SPI. The Python GUI is the host-side tool we use for debugging and sending commands.

## Project layout

**[omnitiles/](omnitiles/)** — Rust firmware for the STM32F767. Handles command parsing, motor control, and communication over UART and SPI. This is the main firmware.

**[dwm_tag/](dwm_tag/)** — Zephyr app for the nRF52 DWM tag. It’s a BLE peripheral (advertises as OmniTile_1) that receives data over the Nordic UART Service and passes it to the STM32 via SPI slave. Used when you’re on the real PCB; you can skip it if you’re on a dev board and use UART only.

**[gui/](gui/)** — Python debug GUI (Viser). Web UI on localhost:8080 to send commands and optionally watch UART output from the STM32. Connects over BLE to the DWM tag or over serial when you pass `--port`.

## How the pieces connect

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

On the PCB: the GUI connects to the nRF52 over BLE. The nRF52 queues incoming packets. When the STM32 is ready, it asserts SPI chip-select; the tag drives DRDY high, the STM32 reads 128 bytes over SPI, then the tag drops DRDY. The STM32 also has UART for debug output (and for direct commands if you’re not using BLE). On a dev board you can ignore the DWM tag and just plug in UART at 115200.

Commands from host to STM32 all use the same format: sync byte `0xA5`, message ID, checksum. See [omnitiles/src/protocol/messages.rs](omnitiles/src/protocol/messages.rs) for the IDs and [omnitiles/src/protocol/parser.rs](omnitiles/src/protocol/parser.rs) for the parser.

## Quick start

- **STM32 firmware:** [omnitiles/README.md](omnitiles/README.md) — Rust toolchain, flash, and how to attach a terminal (`screen` at 115200).
- **DWM tag (nRF52):** [dwm_tag/README.md](dwm_tag/README.md) — building and flashing with the nRF Connect extension in VS Code.
- **GUI:** [gui/README.md](gui/README.md) — run the Python app, BLE vs UART.

## Documentation

Rust API docs: from the repo root, `cd omnitiles && cargo doc --no-deps --open`. The same docs are built in CI and published to GitHub Pages on push to `main`. Protocol message definitions and the parser are in the `omnitiles/src/protocol/` tree.

## License

MIT. See [LICENSE](LICENSE).
