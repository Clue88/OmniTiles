#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};
use nb::block;
use stm32f7xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // GPIOB
    let gpiob = dp.GPIOB.split();
    let mut led = gpiob.pb7.into_push_pull_output(); // PB7 -> LD2
    led.set_low();

    // GPIOC (PC13 = B1 USER, see user manual)
    let gpioc = dp.GPIOC.split();
    let button = gpioc.pc13.into_floating_input();

    // GPIOD (PD8 = TX and PD9 = RX for USART3 via ST-LINK, see user manual)
    let gpiod = dp.GPIOD.split();
    let tx = gpiod.pd8.into_alternate::<7>(); // AF7 = USART3_TX
    let rx = gpiod.pd9.into_alternate::<7>(); // AF7 = USART3_RX

    // Configure USART with 115200 baud rate, everything else default
    let cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let mut serial = Serial::new(dp.USART3, (tx, rx), &clocks, cfg);

    // Initialize SysTick delay for heartbeat
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // Counter
    let mut counter: u8 = 0;
    let mut last_button_state = button.is_high();

    loop {
        // Increment counter and blink on button press
        let current_state = button.is_high();
        if !current_state && last_button_state {
            counter = counter.wrapping_add(1);

            let mut buf = itoa::Buffer::new();
            let s = buf.format(counter);
            for &b in s.as_bytes() {
                block!(serial.write(b)).ok();
            }
            block!(serial.write(b' ')).ok();
            for b in b" OmniTiles!\r\n" {
                block!(serial.write(*b)).ok();
            }

            led.set_high();
            delay.delay_ms(100_u32);
            led.set_low();
        }

        last_button_state = current_state;
        delay.delay_ms(20_u32);
    }
}
