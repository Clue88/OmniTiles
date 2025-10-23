#![no_main]
#![no_std]
#![allow(dead_code)]

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

// General Configuration
const REG_ABS_POS: u8 = 0x01;
const REG_EL_POS: u8 = 0x02;
const REG_MARK: u8 = 0x03;
const REG_SPEED: u8 = 0x04;
const REG_ACC: u8 = 0x05;
const REG_DEC: u8 = 0x06;
const REG_MAX_SPEED: u8 = 0x07;
const REG_MIN_SPEED: u8 = 0x08;
const REG_ADC_OUT: u8 = 0x12;
const REG_OCD_TH: u8 = 0x13;
const REG_FS_SPD: u8 = 0x15;
const REG_STEP_MODE: u8 = 0x16;
const REG_ALARM_EN: u8 = 0x17;
const REG_GATECFG1: u8 = 0x18;
const REG_GATECFG2: u8 = 0x19;
const REG_STATUS: u8 = 0x1B;
const REG_CONFIG: u8 = 0x1A;

// Voltage Mode Configuration
const REG_KVAL_HOLD: u8 = 0x09;
const REG_KVAL_RUN: u8 = 0x0A;
const REG_KVAL_ACC: u8 = 0x0B;
const REG_KVAL_DEC: u8 = 0x0C;
const REG_INT_SPEED: u8 = 0x0D;
const REG_ST_SLP: u8 = 0x0E;
const REG_FN_SLP_ACC: u8 = 0x0F;
const REG_FN_SLP_DEC: u8 = 0x10;
const REG_K_THERM: u8 = 0x11;
const REG_STALL_TH: u8 = 0x14;

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
        10.kHz(),
        &clocks,
        &mut rcc.apb2,
    );

    // SysTick delay
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // Reset device and print initial state
    reset_device(&mut spi, &mut cs);
    let _ = get_status(&mut spi, &mut cs); // Clear any existing errors

    // Configure PowerSTEP01 registers as needed
    set_param(&mut spi, &mut cs, REG_CONFIG, 0x2C10, 2); // SW_MODE = 1, EXT_CLK = 0, OC_SD = 0
    set_param(&mut spi, &mut cs, REG_STEP_MODE, 0x00, 1); // Full step
    set_param(&mut spi, &mut cs, REG_KVAL_HOLD, 0x1D, 1);
    set_param(&mut spi, &mut cs, REG_KVAL_RUN, 0x1D, 1);
    set_param(&mut spi, &mut cs, REG_MARK, 0x64, 3); // 100 steps

    print_str(&mut tx, "PowerSTEP01 initialized with STATUS ");
    print_hex_u16(&mut tx, get_status(&mut spi, &mut cs));
    print_str(&mut tx, "\r\n");
    print_config(&mut tx, &mut spi, &mut cs);
    print_voltage_mode_config(&mut tx, &mut spi, &mut cs);

    let _ = nb::block!(tx.flush());

    let mut is_at_home = true;

    loop {
        let current_state = button.is_high();

        // Toggle between HOME and MARK on button press
        if !current_state && last_button_state {
            if is_at_home {
                print_str(&mut tx, "Going to MARK\r\n");
                go_mark(&mut spi, &mut cs);
                is_at_home = false;
            } else {
                print_str(&mut tx, "Going to HOME\r\n");
                go_home(&mut spi, &mut cs);
                is_at_home = true;
            }

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

/// Print CONFIG, STEP_MODE, and MARK register values
fn print_config<U: Instance, I, P, CS>(
    tx: &mut Tx<U>,
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
) where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    print_str(tx, "  CONFIG: ");
    print_hex_u16(tx, get_param(spi, cs, REG_CONFIG, 2) as u16);
    print_str(tx, "\r\n");
    print_str(tx, "  STEP_MODE: ");
    print_hex_u8(tx, get_param(spi, cs, REG_STEP_MODE, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  MARK: ");
    print_hex_u32(tx, get_param(spi, cs, REG_MARK, 3));
    print_str(tx, "\r\n");
}

/// Print voltage mode configuration register values
fn print_voltage_mode_config<U: Instance, I, P, CS>(
    tx: &mut Tx<U>,
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
) where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    print_str(tx, "  KVAL_HOLD: ");
    print_hex_u8(tx, get_param(spi, cs, REG_KVAL_HOLD, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  KVAL_RUN: ");
    print_hex_u8(tx, get_param(spi, cs, REG_KVAL_RUN, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  KVAL_ACC: ");
    print_hex_u8(tx, get_param(spi, cs, REG_KVAL_ACC, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  KVAL_DEC: ");
    print_hex_u8(tx, get_param(spi, cs, REG_KVAL_DEC, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  INT_SPEED: ");
    print_hex_u16(tx, get_param(spi, cs, REG_INT_SPEED, 2) as u16);
    print_str(tx, "\r\n");
    print_str(tx, "  ST_SLP: ");
    print_hex_u8(tx, get_param(spi, cs, REG_ST_SLP, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  FN_SLP_ACC: ");
    print_hex_u8(tx, get_param(spi, cs, REG_FN_SLP_ACC, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  FN_SLP_DEC: ");
    print_hex_u8(tx, get_param(spi, cs, REG_FN_SLP_DEC, 1) as u8);
    print_str(tx, "\r\n");
    print_str(tx, "  K_THERM: ");
    print_hex_u8(tx, get_param(spi, cs, REG_K_THERM, 1) as u8);
    print_str(tx, "\r\n");
}
