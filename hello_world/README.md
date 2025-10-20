# HelloWorld
Testing playground to gain familiarity with the NUCLEO-F767ZI development board and the
X-NUCLEO-IHM03A1 stepper motor driver. This project includes an (incomplete) driver to program the
powerSTEP01 motor controller over SPI and a debug terminal over USART.

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

Press the USER button (B1) on the devboard to print the PS01 STATUS register to the debug terminal.

To exit the debug terminal, press `Ctrl+A` then `Ctrl+\` then `y`.

You can also use `cargo build --release` and `cargo flash --release` to only build or only flash,
respectively. See https://github.com/burrbull/stm32-template/ for more information about the
template that was used to generate the project skeleton.

## Notes
This example is built using `cortex_m_rt`, which allows applications to be built without using the
Rust standard library for Cortex-M microcontrollers. In order to do so, we provide a `memory.x` file
which describes the device memory layout and include the following scaffolding:

```rs
#![no_main]
#![no_std]

// Some panic handler needs to be included. This one halts the processor on panic.
use panic_halt as _;

use cortex_m_rt::entry;

// Use `main` as the entry point of this application, which may not return.
#[entry]
fn main() -> ! {
    // initialization

    loop {
        // application logic
    }
}
```
