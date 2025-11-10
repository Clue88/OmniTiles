#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, prelude::*};
use stm32f7xx_hal as hal;

mod hw;
use hw::Led;

#[entry]
fn main() -> ! {
    // Peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.freeze();

    // GPIO
    let gpiod = dp.GPIOD.split();

    // LED
    let pd9 = gpiod.pd9.into_push_pull_output();
    let pd10 = gpiod.pd10.into_push_pull_output();
    let mut led_yellow = Led::active_low(pd9);
    let mut led_green = Led::active_low(pd10);

    led_yellow.on();
    led_green.on();

    loop {
        cortex_m::asm::nop();
    }
}
