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

mod powerstep;
use crate::powerstep::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Clocks
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    // LED (PB7)
    let mut led = gpiob.pb7.into_push_pull_output();
    led.set_low();

    // USER button (PC13)
    let button = gpioc.pc13.into_floating_input();
    let mut last_button_state = button.is_high();

    // USART3 via ST-LINK (PD8/PD9)
    let tx = gpiod.pd8.into_alternate::<7>();
    let rx = gpiod.pd9.into_alternate::<7>();
    let cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART3, (tx, rx), &clocks, cfg);
    let (mut tx, _rx) = serial.split();

    // SPI1 (PA5/PA6/PA7), CS = D10 -> PD14
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
        100.kHz(),
        &clocks,
        &mut rcc.apb2,
    );

    const STEP_MODE_REG: u8 = 0x16;
    const STEP_MODE_LEN: u8 = 1;
    const ABS_POS_REG: u8 = 0x01;
    const ABS_POS_LEN: u8 = 3;

    const REG_READ: u8 = 0x03;
    const READ_LEN: u8 = 3;
    const REG_WRITE: u8 = 0x03;
    const WRITE_VAL: u32 = 0x123;
    const WRITE_LEN: u8 = 3;

    // SysTick delay
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // Reset device and print initial state
    reset_device(&mut spi, &mut cs);
    print_str(&mut tx, "PowerSTEP01 initialized with STATUS ");
    print_hex_u16(&mut tx, get_status(&mut spi, &mut cs));
    print_str(&mut tx, ", STEP_MODE ");
    print_hex_u8(
        &mut tx,
        get_param(&mut spi, &mut cs, STEP_MODE_REG, STEP_MODE_LEN) as u8,
    );
    print_str(&mut tx, "\r\n");
    print_str(&mut tx, "ABS_POS: ");
    print_hex_u32(
        &mut tx,
        get_param(&mut spi, &mut cs, ABS_POS_REG, ABS_POS_LEN),
    );
    print_str(&mut tx, "\r\n");

    loop {
        let current_state = button.is_high();
        if !current_state && last_button_state {
            // Read param value
            let read_val = get_param(&mut spi, &mut cs, REG_READ, READ_LEN);
            print_str(&mut tx, "READ  reg ");
            print_hex_u8(&mut tx, REG_READ);
            print_str(&mut tx, "  = ");
            print_hex_u32(&mut tx, read_val);
            print_str(&mut tx, "\r\n");

            // Write param value
            set_param(&mut spi, &mut cs, REG_WRITE, WRITE_VAL, WRITE_LEN);
            print_str(&mut tx, "WRITE reg ");
            print_hex_u8(&mut tx, REG_WRITE);
            print_str(&mut tx, " <= ");
            print_hex_u32(&mut tx, WRITE_VAL);
            print_str(&mut tx, "\r\n");

            // Read param value
            let read_val = get_param(&mut spi, &mut cs, REG_READ, READ_LEN);
            print_str(&mut tx, "READ  reg ");
            print_hex_u8(&mut tx, REG_READ);
            print_str(&mut tx, "  = ");
            print_hex_u32(&mut tx, read_val);
            print_str(&mut tx, "\r\n");

            let _ = nb::block!(tx.flush());

            // Blink
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

/// Print a byte as 0xHH
fn print_hex_u8<U: Instance>(tx: &mut Tx<U>, byte: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let hi = HEX[((byte >> 4) & 0xF) as usize];
    let lo = HEX[(byte & 0xF) as usize];
    let _ = block!(tx.write(b'0'));
    let _ = block!(tx.write(b'x'));
    let _ = block!(tx.write(hi));
    let _ = block!(tx.write(lo));
}

/// Print a 16-bit byte as 0xHHHH
fn print_hex_u16<U: Instance>(tx: &mut Tx<U>, value: u16) {
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
    for b in bytes.iter() {
        let _ = block!(tx.write(*b));
    }
}

/// Print a 32-bit value as 0xHHHH_HHHH
fn print_hex_u32<U: Instance>(tx: &mut Tx<U>, value: u32) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let n = value;
    let bytes = [
        HEX[((n >> 28) & 0xF) as usize],
        HEX[((n >> 24) & 0xF) as usize],
        HEX[((n >> 20) & 0xF) as usize],
        HEX[((n >> 16) & 0xF) as usize],
        HEX[((n >> 12) & 0xF) as usize],
        HEX[((n >> 8) & 0xF) as usize],
        HEX[((n >> 4) & 0xF) as usize],
        HEX[(n & 0xF) as usize],
    ];
    let _ = block!(tx.write(b'0'));
    let _ = block!(tx.write(b'x'));
    for (i, b) in bytes.iter().enumerate() {
        if i == 4 {
            let _ = block!(tx.write(b'_'));
        }
        let _ = block!(tx.write(*b));
    }
}
