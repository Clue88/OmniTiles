#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, prelude::*};
use stm32f7xx_hal as hal;

mod hw;
use hw::led::Led;
use hw::ActiveLevel;

#[entry]
fn main() -> ! {
    // Peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Clocks
    dp.RCC.apb1enr.modify(|_, w| w.tim5en().set_bit());

    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.freeze();

    // GPIO
    let gpioa = dp.GPIOA.split();

    // TIM5 (quadrature encoder)
    let _tim5_ch1 = gpioa.pa0.into_alternate::<2>();
    let _tim5_ch2 = gpioa.pa1.into_alternate::<2>();
    let tim5 = dp.TIM5;
    tim5.smcr.modify(|_, w| w.sms().bits(0b011));

    tim5.ccmr1_input().modify(|_, w| {
        w.cc1s().ti1();
        w.cc2s().ti2();
        w
    });

    tim5.ccer.modify(|_, w| {
        w.cc1e()
            .set_bit()
            .cc1p()
            .clear_bit()
            .cc2e()
            .set_bit()
            .cc2p()
            .clear_bit()
    });

    tim5.arr.write(|w| w.bits(0xFFFF_FFFF));
    tim5.cnt.write(|w| w.bits(0));
    tim5.cr1.modify(|_, w| w.cen().set_bit());

    loop {
        let _position: u32 = tim5.cnt.read().bits();
        cortex_m::asm::nop();
    }
}
