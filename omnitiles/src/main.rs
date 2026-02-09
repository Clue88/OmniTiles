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
    hw::{Adc, BoardPins, ChipSelect, Led, SpiBus, Usart},
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
    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOC, dp.GPIOD, dp.GPIOE, dp.GPIOH);

    // ================================
    // LEDs
    // ================================
    let mut led_yellow = Led::active_low(pins.leds.yellow);
    let mut led_green = Led::active_low(pins.leds.green);

    // ================================
    // USART1 Debug
    // ================================
    let serial = Serial::new(
        dp.USART1,
        (pins.usart1.tx, pins.usart1.rx),
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            ..Default::default()
        },
    );
    let mut usart = Usart::new(serial);

    usart.println("Booting OmniTiles test firmware...");

    // ================================
    // SPI4 + Chip Selects
    // ================================
    let spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnSecondTransition,
        };
        let spi4_raw = Spi::new(dp.SPI4, (pins.spi4.sck, pins.spi4.miso, pins.spi4.mosi));
        let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi4_enabled)
    };
    let cs1 = ChipSelect::active_low(pins.spi4.cs1);
    let cs2 = ChipSelect::active_low(pins.spi4.cs2);

    // ================================
    // ADC1 Current Sense
    // ================================
    let adc1 = RefCell::new(Adc::adc1(dp.ADC1));

    // ================================
    // P16 Linear Actuator
    // ================================
    let mut p16 = ActuonixLinear::new(
        Drv8873::new(cs1),
        pins.m1.in1,
        pins.m1.in2,
        pins.m1.nsleep,
        pins.m1.disable,
        Adc::make_reader(&adc1, 8), // ADC1_IN8, TODO: update based on actual wiring
        150.0,
    );
    p16.enable_outputs();

    // ================================
    // P16 Track Actuator
    // ================================
    let mut t16 = ActuonixLinear::new(
        Drv8873::new(cs2),
        pins.m2.in1,
        pins.m2.in2,
        pins.m2.nsleep,
        pins.m2.disable,
        Adc::make_reader(&adc1, 12), // ADC1_IN12, TODO: update based on actual wiring
        100.0,
    );
    t16.enable_outputs();

    // ================================
    // Parser + Telemetry Ticker
    // ================================
    let mut parser = Parser::new();
    let mut ticker = 0u32;

    usart.println("System Ready. Waiting for commands over USART...");

    loop {
        // Poll USART for incoming bytes
        if let Some(byte) = usart.read_byte() {
            if let Some(command) = parser.push(byte) {
                match command {
                    // P16 Controls
                    Command::P16Extend => {
                        p16.extend();
                        led_green.on();
                    }
                    Command::P16Retract => {
                        p16.retract();
                        led_green.on();
                    }
                    Command::P16Brake => {
                        p16.brake();
                        led_green.off();
                    }

                    // T16 Controls
                    Command::T16Extend => {
                        t16.extend();
                        led_yellow.on();
                    }
                    Command::T16Retract => {
                        t16.retract();
                        led_yellow.on();
                    }
                    Command::T16Brake => {
                        t16.brake();
                        led_yellow.off();
                    }
                }
            }
        }

        // Periodic telemetry
        delay.delay_ms(10_u32);
        ticker += 1;
        if ticker >= 20 {
            ticker = 0;
            writeln!(
                usart,
                "STATUS_P16 {:.2} {} false\r",
                p16.position_mm(),
                p16.position_raw()
            )
            .ok();
            writeln!(
                usart,
                "STATUS_T16 {:.2} {} false\r",
                t16.position_mm(),
                t16.position_raw()
            )
            .ok();
        }
    }
}
