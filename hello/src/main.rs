#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use hal::{pac, prelude::*};
use stm32f7xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // GPIOB (LD1=PB0, LD2=PB7, LD3=PB14)
    let gpiob = dp.GPIOB.split();
    let mut ld1 = gpiob.pb0.into_push_pull_output(); // green
    let mut ld2 = gpiob.pb7.into_push_pull_output(); // blue
    let mut ld3 = gpiob.pb14.into_push_pull_output(); // red

    ld1.set_low();
    ld2.set_low();
    ld3.set_low();

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    loop {
        // LD1 (green)
        ld1.set_high();
        delay.delay_ms(300_u32);
        ld1.set_low();
        delay.delay_ms(100_u32);

        // LD2 (blue)
        ld2.set_high();
        delay.delay_ms(300_u32);
        ld2.set_low();
        delay.delay_ms(100_u32);

        // LD3 (red)
        ld3.set_high();
        delay.delay_ms(300_u32);
        ld3.set_low();

        delay.delay_ms(400_u32);
    }
}
