#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, prelude::*};
use stm32f7xx_hal as hal;

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    loop {}
}
