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
    can::Can,
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

use omnitiles::{
    drivers::{ActuonixLinear, Drv8873, Fit0185},
    hw::{adc::volts_from_adc, Adc, BoardPins, CanBus, ChipSelect, Encoder, Led, SpiBus, Usart},
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
    let mut spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnSecondTransition,
        };
        let spi4_raw = Spi::new(dp.SPI4, (pins.spi4.sck, pins.spi4.miso, pins.spi4.mosi));
        let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi4_enabled)
    };
    let cs1 = ChipSelect::active_low(pins.spi4.cs1);

    // ================================
    // CAN2 (Loopback)
    // ================================
    let pclk1_hz = clocks.pclk1().to_Hz();
    writeln!(usart, "PCLK1 = {} Hz\r", pclk1_hz).ok();
    const CAN_BTR: u32 = 0x001C_0003; // 250 kbps @ 16 MHz

    let mut can2_hal = Can::new(dp.CAN2, &mut apb1, (pins.can2.tx, pins.can2.rx));

    // For dual CAN filter setup, we need to configure filters on CAN1
    {
        let can1_hal = Can::new(dp.CAN1, &mut apb1, (pins.can1.tx, pins.can1.rx));
        let mut can1_bus = CanBus::new(can1_hal, CAN_BTR, false, false);
        can1_bus.configure_accept_all_filters_for_dual_can(&mut can2_hal);
    }

    let mut can_bus = CanBus::new(can2_hal, CAN_BTR, true, false); // loopback mode

    // ================================
    // ADC1 Current Sense
    // ================================
    let adc1 = RefCell::new(Adc::adc1(dp.ADC1));

    let mut read_m1_iprop1 = Adc::make_reader(&adc1, 14);
    let mut read_m1_iprop2 = Adc::make_reader(&adc1, 15);

    // ================================
    // P16 Linear Actuator
    // ================================
    let mut p16 = ActuonixLinear::new(
        Drv8873::new(cs1),
        pins.m1.in1,
        pins.m1.in2,
        pins.m1.nsleep,
        pins.m1.disable,
        Adc::make_reader(&adc1, 8), // ADC1_IN8, update based on actual wiring
        150.0,                      // stroke length in mm
    );
    p16.enable_outputs();

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
                    Command::P16Extend => {
                        usart.println("CMD: Extend");
                        p16.extend();
                        led_green.on();
                    }
                    Command::P16Retract => {
                        usart.println("CMD: Retract");
                        p16.retract();
                        led_yellow.on();
                    }
                    Command::P16Brake => {
                        usart.println("CMD: Brake");
                        p16.brake();
                        led_green.off();
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

            let pos_mm = p16.position_mm();
            let raw_adc = p16.position_raw();

            writeln!(usart, "STATUS {:.2} {} false\r", pos_mm, raw_adc).ok();
        }
    }
}
