#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

mod hw;
use hw::{ChipSelect, Led, SpiBus, Usart};

#[entry]
fn main() -> ! {
    // Peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb2 = rcc.apb2;

    // GPIO
    let gpioa = dp.GPIOA.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // LED
    let mut led_yellow = Led::active_low(gpiod.pd9);
    let mut led_green = Led::active_low(gpiod.pd10);

    // USART1 (DBG)
    let tx = gpioa.pa9.into_alternate::<7>();
    let rx = gpioa.pa10.into_alternate::<7>();
    let usart_cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART1, (tx, rx), &clocks, usart_cfg);
    let mut usart = Usart::new(serial);

    // SPI4
    let sck = gpioe.pe12.into_alternate::<5>();
    let miso = gpioe.pe13.into_alternate::<5>();
    let mosi = gpioe.pe14.into_alternate::<5>();
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi4_raw = Spi::new(dp.SPI4, (sck, miso, mosi));
    let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
    let mut spi_bus = SpiBus::new(spi4_enabled);

    let mut cs1 = ChipSelect::active_low(gpioe.pe4);
    let mut cs2 = ChipSelect::active_low(gpioe.pe11);

    // EXAMPLE USAGES
    led_yellow.on();
    led_green.on();

    usart.println("Hello world!");

    cs1.select();
    let _ = spi_bus.transfer_byte(0x67);
    cs1.deselect();
    cs2.select();
    let _ = spi_bus.transfer_byte(0xFE);
    cs2.deselect();

    loop {
        cortex_m::asm::nop();
    }
}
