#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use hal::{
    pac,
    prelude::*,
    serial::{Config, Instance, Serial, Tx},
    spi::{Mode, Phase, Polarity, Spi},
};
use nb::block;
use stm32f7xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clocks
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // GPIO pins
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    // LEDs
    let mut led = gpiob.pb7.into_push_pull_output(); // PB7 -> LD2
    led.set_low();

    // USER button
    let button = gpioc.pc13.into_floating_input();
    let mut last_button_state = button.is_high();

    // USART3 via ST-LINK
    let tx = gpiod.pd8.into_alternate::<7>();
    let rx = gpiod.pd9.into_alternate::<7>();

    let cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART3, (tx, rx), &clocks, cfg);
    let (mut tx, _rx) = serial.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate::<5>();
    let miso = gpioa.pa6.into_alternate::<5>();
    let mosi = gpioa.pa7.into_alternate::<5>();
    let mut cs = gpioa.pa4.into_push_pull_output();

    cs.set_high();
    const SPI_MODE: Mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let mut spi = Spi::new(dp.SPI1, (sck, miso, mosi)).enable::<u8>(
        SPI_MODE,
        1.MHz(),
        &clocks,
        &mut rcc.apb2,
    );

    // Initialize SysTick delay for heartbeat
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    loop {
        // Send loopback test on button press
        let current_state = button.is_high();
        if !current_state && last_button_state {
            let mut buf = [0xDE, 0xAD, 0xBE, 0xEF];
            cs.set_low();
            let _ = spi.transfer(&mut buf);
            cs.set_high();

            if buf == [0xDE, 0xAD, 0xBE, 0xEF] {
                print_str(&mut tx, "SPI Connected: ");
            } else {
                print_str(&mut tx, "SPI Disconnected: ");
            }
            print_hex_array(&mut tx, &buf);
            print_str(&mut tx, "\r\n");

            led.set_high();
            delay.delay_ms(100_u32);
            led.set_low();
        }

        last_button_state = current_state;
        delay.delay_ms(20_u32);
    }
}

/// Send a &str over UART TX
fn print_str<U: Instance>(tx: &mut Tx<U>, s: &str) {
    for &b in s.as_bytes() {
        let _ = block!(tx.write(b));
    }
}

/// Print bytes as hex, space-separated
fn print_hex_array<U: Instance>(tx: &mut Tx<U>, data: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, &byte) in data.iter().enumerate() {
        let hi = HEX[(byte >> 4) as usize];
        let lo = HEX[(byte & 0xF) as usize];
        let _ = block!(tx.write(hi));
        let _ = block!(tx.write(lo));
        if i + 1 < data.len() {
            let _ = block!(tx.write(b' '));
        }
    }
}
