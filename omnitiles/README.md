# OmniTiles
Embedded systems code for Fall 2025 ESE Senior Design.

## Installation
The following instructions are designed for macOS, but will probably also work on any other
Unix-like OS like Linux.

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

You can also use `cargo build --release` and `cargo flash --release` to only build or only flash,
respectively. See https://github.com/burrbull/stm32-template/ for more information about the
template that was used to generate the project skeleton.

### USB-UART Bridge for RP2040
OmniTiles PCB v1 has an RP2040 (Raspberry Pi Pico) connected to the MCU over UART. In order to use
the debug terminal, it is necessary to flash a USB-UART program to the RP2040 (not needed for dev
boards).

1. Download the .uf2 file from the
[pico-uart-bridge repo](https://github.com/Noltari/pico-uart-bridge/releases/tag/v4.2).
2. While pressing the BOOT button on the RP2040, connect the board to a computer with a USB cable,
then release the button.
3. The RP2040 will now appear as a storage volume named `RPI-RP2`. Drag and drop the .uf2 file onto
the drive. It will automatically disconnect.

This should only be necessary once—after flashing the RP2040, you can use the debug terminal as
normal.

## Documentation
Documentation can be automatically generated using `rustdoc`. To regenerate the documentation in
`/doc` and update the documentation hosted on GitHub Pages, run:
```bash
cargo doc --no-deps --target-dir doc
```
