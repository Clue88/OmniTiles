#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};
use stm32f7xx_hal as hal;

mod hw;
use hw::{Led, Usart};

#[entry]
fn main() -> ! {
    // Peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // GPIO
    let gpioa = dp.GPIOA.split();
    let gpiod = dp.GPIOD.split();

    // LED
    let mut led_yellow = Led::active_low(gpiod.pd9);
    let mut led_green = Led::active_low(gpiod.pd10);

    led_yellow.on();
    led_green.on();

    // USART1 (DBG)
    let usart_tx = gpioa.pa9.into_alternate::<7>();
    let usart_rx = gpioa.pa10.into_alternate::<7>();
    let usart_cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART1, (usart_tx, usart_rx), &clocks, usart_cfg);
    let mut usart = Usart::new(serial);

    usart.println("Hello world!");

    loop {
        cortex_m::asm::nop();
    }
}
