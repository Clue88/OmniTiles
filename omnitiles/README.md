# OmniTiles firmware

Rust firmware for the main MCU (STM32F7). It parses commands from the host over SPI (when the
[DWM tag](../dwm_tag/) is present) or over UART, and drives the actuators and debug output. For how
this fits with the DWM tag and the [GUI](../gui/), see the [root README](../README.md).

See the [generated documentation](https://clue88.github.io/OmniTiles/omnitiles/index.html) for more
details on the Rust firmware for this project.

## Installation

Written for macOS; should work on other Unix-like systems.

Install Rust if needed:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Then:

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

## Usage

```bash
cargo run --release
```

To watch debug output, attach to the UART at 115200, e.g.:

```bash
screen /dev/tty.usbmodem* 115200
```

Exit screen with `Ctrl+A`, then `Ctrl+\`, then `y`. You can also use `cargo build --release` and
`cargo flash --release` separately. More on the template:
[stm32-template](https://github.com/burrbull/stm32-template/).

### USB–UART bridge (RP2040) on PCB v1

The PCB has an RP2040 (Pico) in front of the MCU UART. To get a debug serial port you need to flash
a USB–UART firmware on the RP2040 once (not needed on a bare dev board). Download the `.uf2` from
[pico-uart-bridge](https://github.com/Noltari/pico-uart-bridge/releases/tag/v4.2), hold BOOT on the
RP2040, plug USB, release BOOT, then drag the `.uf2` onto the `RPI-RP2` volume. After that the debug
terminal works as above.

## Documentation

```bash
cargo doc --no-deps --open
```

CI builds these docs and deploys them to GitHub Pages on push to `main`.
