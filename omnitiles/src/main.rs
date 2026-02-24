// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#![no_main]
#![no_std]

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

/// Map protocol speed byte (0–255) to motor set_speed magnitude in [0.0, 1.0].
fn speed_to_float(speed: u8) -> f32 {
    (speed as f32) / 255.0
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb2 = rcc.apb2;

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOC, dp.GPIOD);

    // LEDs are active-high on the F767ZI devboard but active-low on PCB v1
    let mut led_blue = Led::active_high(pins.leds.blue);
    let mut led_green = Led::active_high(pins.leds.green);

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

    let adc1 = RefCell::new(Adc::adc1(dp.ADC1));

    let pwm = dp.TIM3.pwm::<_, _, 1_000_000>(
        (pins.m1.in1, pins.m1.in2, pins.m2.in1, pins.m2.in2),
        50.micros(), // 20 kHz
        &clocks,
    );
    let (m1_in1, m1_in2, m2_in1, m2_in2) = pwm.split();

    let mut m1 = ActuonixLinear::new(
        Drv8873::new(ChipSelect::active_low(pins.m1.cs)),
        m1_in1,
        m1_in2,
        pins.m1.nsleep,
        pins.m1.disable,
        Adc::make_reader(&adc1, 9), // ADC1_IN9
        150.0,                      // P16 has 150 mm stroke length
        5.0,                        // 5 mm software buffer at each end
    );
    m1.enable_outputs();

    let mut m2 = ActuonixLinear::new(
        Drv8873::new(ChipSelect::active_low(pins.m2.cs)),
        m2_in1,
        m2_in2,
        pins.m2.nsleep,
        pins.m2.disable,
        Adc::make_reader(&adc1, 12), // ADC1_IN12
        100.0,                       // T16 has 100 mm stroke length
        5.0,                         // 5 mm software buffer at each end
    );
    m2.enable_outputs();

    let mut parser = Parser::new();

    loop {
        m1.enforce_limits();
        m2.enforce_limits();

        if drdy.is_high() {
            delay.delay_ms(2_u32);

            let mut buf = [0u8; 128];

            // Prepare telemetry packet (P16 = m1, T16 = m2)
            let p16_raw = m1.position_raw();
            let t16_raw = m2.position_raw();

            // Scale 12-bit ADC (0-4095) down to 8-bit (0-255)
            let p16_pos = (p16_raw >> 4) as u8;
            let t16_pos = (t16_raw >> 4) as u8;

            buf[0] = omnitiles::protocol::messages::START_BYTE;
            buf[1] = omnitiles::protocol::messages::MSG_TELEMETRY;
            buf[2] = p16_pos;
            buf[3] = t16_pos;
            buf[4] = buf[1].wrapping_add(buf[2]).wrapping_add(buf[3]);

            cs.select();
            delay.delay_us(50_u32);
            spi_bus.transfer_in_place(&mut buf).unwrap_or_default();
            delay.delay_us(50_u32);
            cs.deselect();

            let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
            let payload = &buf[..len];
            for &byte in payload {
                if let Some(cmd) = parser.push(byte) {
                    match cmd {
                        Command::Ping => {
                            writeln!(usart, "cmd: PING — System is alive.\r").ok();
                        }
                        Command::M1Extend(speed) => {
                            writeln!(usart, "cmd: M1Extend speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m1.set_speed(s);
                            led_green.on();
                        }
                        Command::M1Retract(speed) => {
                            writeln!(usart, "cmd: M1Retract speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m1.set_speed(-s);
                            led_green.on();
                        }
                        Command::M1Brake => {
                            writeln!(usart, "cmd: M1Brake\r").ok();
                            m1.brake();
                            led_green.off();
                        }
                        Command::M2Extend(speed) => {
                            writeln!(usart, "cmd: M2Extend speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m2.set_speed(s);
                            led_blue.on();
                        }
                        Command::M2Retract(speed) => {
                            writeln!(usart, "cmd: M2Retract speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m2.set_speed(-s);
                            led_blue.on();
                        }
                        Command::M2Brake => {
                            writeln!(usart, "cmd: M2Brake\r").ok();
                            m2.brake();
                            led_blue.off();
                        }
                    }
                }
            }

            while drdy.is_high() {}
        }

        if let Some(byte) = usart.read_byte() {
            if let Some(cmd) = parser.push(byte) {
                match cmd {
                    Command::Ping => {
                        writeln!(usart, "cmd: PING — System is alive.\r").ok();
                    }
                    Command::M1Extend(speed) => {
                        writeln!(usart, "cmd: M1Extend speed={}\r", speed).ok();
                        let s = speed_to_float(speed);
                        m1.set_speed(s);
                        led_green.on();
                    }
                    Command::M1Retract(speed) => {
                        writeln!(usart, "cmd: M1Retract speed={}\r", speed).ok();
                        let s = speed_to_float(speed);
                        m1.set_speed(-s);
                        led_green.on();
                    }
                    Command::M1Brake => {
                        writeln!(usart, "cmd: M1Brake\r").ok();
                        m1.brake();
                        led_green.off();
                    }
                    Command::M2Extend(speed) => {
                        writeln!(usart, "cmd: M2Extend speed={}\r", speed).ok();
                        let s = speed_to_float(speed);
                        m2.set_speed(s);
                        led_blue.on();
                    }
                    Command::M2Retract(speed) => {
                        writeln!(usart, "cmd: M2Retract speed={}\r", speed).ok();
                        let s = speed_to_float(speed);
                        m2.set_speed(-s);
                        led_blue.on();
                    }
                    Command::M2Brake => {
                        writeln!(usart, "cmd: M2Brake\r").ok();
                        m2.brake();
                        led_blue.off();
                    }
                }
            }
        }
    }
}
