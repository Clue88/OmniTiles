# Hello

## Usage
```bash
cargo run --release
```

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
