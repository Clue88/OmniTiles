# OmniTiles firmware

Embedded firmware for the OmniTiles robotics platform: runs on the **main MCU (STM32F767)**, parses commands from the host (over SPI from the [DWM tag](../dwm_tag/) or over UART), and drives actuators (e.g. P16 linear, T16 track) and debug output.

This crate is the core of the project. For an overview of how it fits with the DWM tag and the [GUI](../gui/), see the [repository root README](../README.md).

## Crate structure

| Module | Purpose |
| ------ | -------- |
| [`hw`](src/hw/) | Board support: pin mappings (dev board and PCB v1), USART, SPI, CAN, ADC, encoder, LED. |
| [`drivers`](src/drivers/) | Device drivers: DRV8873 (SPI motor driver), Actuonix linear actuator, Fit0185, Gim6010. |
| [`motors`](src/motors/) | Actuator abstractions: lift (linear) and tilt (track), built on the drivers. |
| [`control`](src/control/) | Control algorithms: PID, lift controller. |
| [`protocol`](src/protocol/) | Command protocol: [message IDs and `Command` enum](src/protocol/messages.rs), [byte parser](src/protocol/parser.rs) (sync byte `0xA5`, ID, checksum). |

The application entrypoint is [`src/main.rs`](src/main.rs): it initializes pins, USART, SPI (with chip-select and DRDY), and runs a loop that reads from SPI when DRDY is high and from UART, feeds bytes into the protocol parser, and executes commands (e.g. Ping, P16/T16 extend/retract/brake).

## Installation

The following instructions are designed for macOS, but will probably also work on any other Unix-like OS like Linux.

Install Rust (if not already installed):

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install the required dependencies:

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

## Usage

```bash
cargo run --release
```

To run the debug terminal, open a `screen` session with baud rate 115200:

```bash
screen /dev/tty.usbmodem* 115200
```

To exit the debug terminal, press `Ctrl+A` then `Ctrl+\` then `y`.

You can also use `cargo build --release` and `cargo flash --release` to only build or only flash, respectively. See the [stm32-template](https://github.com/burrbull/stm32-template/) for more information about the template used to generate the project skeleton.

### USB–UART bridge (RP2040) on PCB v1

OmniTiles PCB v1 has an RP2040 (Raspberry Pi Pico) connected to the MCU over UART. To use the debug terminal, flash a USB–UART program to the RP2040 (not needed for dev boards).

1. Download the `.uf2` file from the [pico-uart-bridge releases](https://github.com/Noltari/pico-uart-bridge/releases/tag/v4.2).
2. Hold the BOOT button on the RP2040, connect the board via USB, then release the button.
3. The RP2040 appears as a storage volume `RPI-RP2`. Drag and drop the `.uf2` file onto it; it will disconnect when done.

This is typically needed only once; afterward you can use the debug terminal as usual.

## Documentation

API documentation is generated with `rustdoc`:

```bash
cargo doc --no-deps --open
```

CI builds these docs on pushes to `main` and deploys them to GitHub Pages.
