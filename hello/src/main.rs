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
    let mut cs = gpiod.pd14.into_push_pull_output();
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
        // Get powerSTEP01 status on button press
        let current_state = button.is_high();
        if !current_state && last_button_state {
            let status = powerstep_get_status(&mut spi, &mut cs, &mut tx);

            print_str(&mut tx, "PS01 STATUS: ");
            print_u16_hex(&mut tx, status);
            print_str(&mut tx, "\r\n");
            let _ = nb::block!(tx.flush());

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

/// Print a 16-bit value as 0xHHHH
fn print_u16_hex<U: Instance>(tx: &mut Tx<U>, value: u16) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let n = value;
    let bytes = [
        HEX[((n >> 12) & 0xF) as usize],
        HEX[((n >> 8) & 0xF) as usize],
        HEX[((n >> 4) & 0xF) as usize],
        HEX[(n & 0xF) as usize],
    ];
    let _ = block!(tx.write(b'0'));
    let _ = block!(tx.write(b'x'));
    for b in bytes {
        let _ = block!(tx.write(b));
    }
}

/// Send the contents of `data` over SPI and read incoming data back into the buffer
fn spi_xfer_in_place<I, P>(spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>, data: &mut [u8])
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
{
    for b in data.iter_mut() {
        let _ = nb::block!(spi.send(*b));
        match nb::block!(spi.read()) {
            Ok(rx) => *b = rx,
            Err(_) => { /* leave byte unchanged on error */ }
        }
    }
}

/// Send the GetStatus command to powerSTEP01 and return status register value
fn powerstep_get_status<I, P, U>(
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut hal::gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>,
    _tx: &mut Tx<U>,
) -> u16
where
    I: hal::spi::Instance,
    U: Instance,
    P: hal::spi::Pins<I>,
{
    // 0xD0 is the GetStatus command, and the register value is placed in the following 2 bytes
    let mut buf = [0xD0, 0x00, 0x00];

    cs.set_low();
    spi_xfer_in_place(spi, &mut buf);
    cs.set_high();

    ((buf[1] as u16) << 8) | (buf[2] as u16)
}
