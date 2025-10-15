#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, prelude::*};
use stm32f7xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // RCC / clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // PB7 = LD2 (blue)
    let gpiob = dp.GPIOB.split();
    let mut led = gpiob.pb7.into_push_pull_output();

    // SysTick delay from cortex-m, needs core clock in Hz (u32)
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    loop {
        led.set_high();
        delay.delay_ms(300_u32);
        led.set_low();
        delay.delay_ms(300_u32);
    }
}
