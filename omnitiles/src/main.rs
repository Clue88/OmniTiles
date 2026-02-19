// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#![no_main]
#![no_std]
#![allow(unused)]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;

use hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

use omnitiles::{
    drivers::{ActuonixLinear, Drv8873},
    hw::{pins_f767zi::BoardPins, Adc, ChipSelect, Led, SpiBus, Usart},
    protocol::{Command, Parser},
};

#[entry]
fn main() -> ! {
    // ================================
    // Peripherals + Clocks + SysTick
    // ================================
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb1 = rcc.apb1;
    let mut apb2 = rcc.apb2;

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // ================================
    // Board Pins
    // ================================
    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOC, dp.GPIOD);

    // ================================
    // LEDs
    // ================================
    let mut led_blue = Led::active_low(pins.leds.blue);
    let mut led_green = Led::active_low(pins.leds.green);

    // ================================
    // USART3 Debug
    // ================================
    let serial = Serial::new(
        dp.USART3,
        (pins.usart3.tx, pins.usart3.rx),
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            ..Default::default()
        },
    );
    let mut usart = Usart::new(serial);

    usart.println("Booting OmniTiles firmware...");

    // ================================
    // SPI1 + Chip Selects
    // ================================
    let mut spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi1_raw = Spi::new(dp.SPI1, (pins.spi1.sck, pins.spi1.miso, pins.spi1.mosi));
        let spi1_enabled = spi1_raw.enable::<u8>(spi_mode, 100.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi1_enabled)
    };
    let mut cs = ChipSelect::active_low(pins.spi1.cs);
    let drdy = pins.spi1.drdy;

    cs.deselect();

    loop {
        while drdy.is_low() {}
        delay.delay_ms(2_u32);

        let mut buf = [0u8; 128];
        cs.select();
        delay.delay_us(50_u32);
        spi_bus.transfer_in_place(&mut buf).unwrap_or_default();
        delay.delay_us(50_u32);
        cs.deselect();

        let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        if len > 0 {
            if let Ok(text) = core::str::from_utf8(&buf[..len]) {
                writeln!(usart, "RX: {}\r", text).ok();
            } else {
                writeln!(usart, "RX: {:02x?}\r", &buf[..8.min(len)]).ok();
            }
        } else {
            writeln!(usart, "RX: Empty (All 0s)\r").ok();
        }

        while drdy.is_high() {}
    }
}
