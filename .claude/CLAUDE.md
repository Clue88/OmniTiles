# OmniTiles

Robotics testing platform for ESE Senior Design (2025-26). Multi-MCU system: STM32F7
(Rust firmware) + DWM3001CDK nRF52 tag (Zephyr, BLE-to-SPI bridge) + nRF52 UWB anchors
(Zephyr, DS-TWR ranging) + Python debug GUI (Viser).

See [README.md](../README.md) for the system diagram and protocol overview.

## Project layout

| Directory | Language | Build | What it does |
|-----------|----------|-------|--------------|
| `omnitiles/` | Rust (no_std) | `cargo run --release` | STM32F7 firmware: command parsing, motor control, sensors |
| `dwm_tag/` | C (Zephyr) | nRF Connect for VS Code | nRF52 BLE-to-SPI bridge + UWB ranging initiator |
| `dwm_anchor/` | C (Zephyr) | nRF Connect for VS Code | nRF52 UWB anchor (DS-TWR responder) |
| `gui/` | Python 3.12+ | `uv sync && uv run main.py` | Viser web UI on localhost:8080 |
| `zephyr-dw3000-decadriver/` | C | Zephyr module | DW3000 UWB chip driver (external) |

## Hardware

- Multiple tiles, each with a custom PCB (STM32F7 + DWM3001CDK nRF52 via SPI)
- Motor drivers controlled via IN1/IN2 pins (see `omnitiles/src/main.rs`)
- Additional sensors over SPI or I2C (e.g., VL53L0x ToF)
- Boards are flashed via ST-LINK or J-LINK; the GUI runs on the host laptop
- UWB anchors: DWM3001CDK boards at fixed positions, shared by all tiles
- "OmniTile_1" is a placeholder name; each tile will need a unique BLE identity

## Code style

- **Rust**: `rustfmt`. No `embedded_hal` directly; use `stm32f7xx-hal` APIs (the HAL
  re-exports an older `embedded_hal` version that conflicts with the latest).
- **C (Zephyr)**: clang-format, Google style base. See `.clang-format` or the config in
  subdirectory READMEs.
- **Python**: `black`.
- Keep code clean. Don't add comments that restate what the code already says. Don't add
  emojis. Don't add docstrings or type annotations to code you didn't change.

## How I work

- Changes should be small, atomic, and verifiable.
- I test on real hardware. For each feature, propose a minimal test plan.
- Add log statements (temporarily if very verbose) as needed for verification.
- Don't add unnecessary dependencies.
- Don't refactor or "improve" code beyond what was asked.

## Binary protocol

All components share the same packet format: `[0xA5] [msg_id] [payload?] [checksum]`.
Definitions are in `omnitiles/src/protocol/messages.rs`; parser in
`omnitiles/src/protocol/parser.rs`. The GUI, tag, and firmware all speak this protocol.
