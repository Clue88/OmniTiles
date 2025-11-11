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
    let pin_led_yellow = gpiod.pd9.into_push_pull_output();
    let pin_led_greem = gpiod.pd10.into_push_pull_output();
    let mut led_yellow = Led::active_low(pin_led_yellow);
    let mut led_green = Led::active_low(pin_led_greem);

    led_yellow.on();
    led_green.on();

    // USART1 (DBG)
    let pin_usart_tx = gpioa.pa9.into_alternate::<7>();
    let pin_usart_rx = gpioa.pa10.into_alternate::<7>();
    let usart_cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART1, (pin_usart_tx, pin_usart_rx), &clocks, usart_cfg);
    let (usart_tx, _usart_rx) = serial.split();
    let mut usart = Usart::new(usart_tx);

    usart.println("Hello world!");

    loop {
        cortex_m::asm::nop();
    }
}
