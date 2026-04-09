// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

#![no_main]
#![no_std]
#![allow(unused)]

use cortex_m::delay::Delay;
use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;
use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;

use hal::{
    i2c::{BlockingI2c, Mode as I2cMode},
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

#[cfg(feature = "mobile-base")]
use omnitiles::{control::BaseController, drivers::Tb6612};
use omnitiles::{
    control::{LinearController, LinearMode, Pid},
    drivers::{ActuonixLinear, Drv8873, Vl53l0x},
    hw::{pins_v2::BoardPins, spi::NoChipSelect, Adc, ChipSelect, I2cBus, Led, SpiBus, Usart},
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
    let mut apb1 = rcc.apb1;
    let mut apb2 = rcc.apb2;

    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    let sysclk_hz = clocks.sysclk().raw() as f32;
    {
        let mut dcb = cp.DCB;
        let mut dwt = cp.DWT;
        dcb.enable_trace();
        DWT::unlock();
        dwt.enable_cycle_counter();
    }

    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOC, dp.GPIOD, dp.GPIOE);

    // LEDs are active-high on the F767ZI devboard but active-low on PCB v1
    let mut led_red = Led::active_low(pins.leds.red);
    let mut led_yellow = Led::active_low(pins.leds.yellow);
    let mut led_green = Led::active_low(pins.leds.green);

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

    usart.println("Booting OmniTiles firmware...");

    let i2c_raw = BlockingI2c::i2c1(
        dp.I2C1,
        (pins.i2c1.scl, pins.i2c1.sda),
        I2cMode::standard(100_000_u32.Hz()),
        &clocks,
        &mut apb1,
        10_000, // data_timeout_us
    );
    let i2c_bus = I2cBus::new(i2c_raw);
    let mut tof = Vl53l0x::new(i2c_bus)
        .and_then(|mut s| {
            s.static_init()?;
            s.load_tuning()?;
            s.calibrate()?;
            Ok(s)
        })
        .ok();
    if tof.is_some() {
        usart.println("ToF: VL53L0X initialized");
    } else {
        usart.println("ToF: not detected, skipping");
    }

    let mut spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi4_raw = Spi::new(dp.SPI4, (pins.spi4.sck, pins.spi4.miso, pins.spi4.mosi));
        let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 100.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi4_enabled)
    };
    let mut cs1 = ChipSelect::active_low(pins.spi4.cs1);
    let drdy = pins.spi4.drdy;
    cs1.deselect();

    let mut cs2 = ChipSelect::active_low(pins.spi4.cs2);
    cs2.deselect();

    let adc1 = RefCell::new(Adc::adc1(dp.ADC1));

    let pwm = dp.TIM3.pwm::<_, _, 1_000_000>(
        (pins.m1.in1, pins.m1.in2, pins.m2.in1, pins.m2.in2),
        50.micros(), // 20 kHz
        &clocks,
    );
    let (m1_in1, m1_in2, m2_in1, m2_in2) = pwm.split();

    let mut m1_actuator = ActuonixLinear::new(
        Drv8873::new(NoChipSelect),
        m1_in1,
        m1_in2,
        pins.m1.nsleep,
        pins.m1.disable,
        Adc::make_reader(&adc1, 8), // ADC1_IN8
        150.0,                      // P16 has 150 mm stroke length
        20.0,                       // 20 mm buffer at bottom (retracted)
        35.0,                       // 35 mm buffer at top (extended)
    );
    m1_actuator.enable_outputs();
    let mut m1 = LinearController::new(
        m1_actuator,
        Pid::new(0.0, 5.0, 0.0), // Under load we might need to bring back kp
        20.0,                    // min_position_mm (buffer at retracted end)
        115.0,                   // max_position_mm (stroke 150 mm - buffer 35 mm at extended end)
        2.0,                     // on_target_tolerance_mm
    );

    let mut m2_actuator = ActuonixLinear::new(
        Drv8873::new(NoChipSelect),
        m2_in1,
        m2_in2,
        pins.m2.nsleep,
        pins.m2.disable,
        Adc::make_reader(&adc1, 12), // ADC1_IN12
        100.0,                       // T16 has 100 mm stroke length
        25.0,                        // 25 mm buffer at bottom (retracted)
        15.0,                        // 15 mm buffer at top (extended)
    );
    m2_actuator.enable_outputs();
    let mut m2 = LinearController::new(
        m2_actuator,
        Pid::new(0.0, 5.0, 0.0), // Under load we might need to bring back kp
        25.0,                    // min_position_mm (buffer at retracted end)
        85.0,                    // max_position_mm (stroke 100 mm - buffer 15 mm at extended end)
        0.45,                    // on_target_tolerance_mm
    );

    // Disable PID control and engage brakes at boot
    m1.mode = LinearMode::Disabled;
    m2.mode = LinearMode::Disabled;
    m1.actuator.brake();
    m2.actuator.brake();
    led_red.off();
    led_yellow.off();
    led_green.off();

    #[cfg(feature = "mobile-base")]
    let mut base = {
        let mut stby_a = pins.wheels.stby_a;
        let mut stby_b = pins.wheels.stby_b;
        stby_a.set_high();
        stby_b.set_high();

        let wheel_pwm = dp.TIM4.pwm::<_, _, 1_000_000>(
            (
                pins.wheels.w1_pwm,
                pins.wheels.w2_pwm,
                pins.wheels.w3_pwm,
                pins.wheels.w4_pwm,
            ),
            50.micros(), // 20 kHz
            &clocks,
        );
        let (w1_pwm, w2_pwm, w3_pwm, w4_pwm) = wheel_pwm.split();

        let mut base = BaseController::new(
            Tb6612::new(pins.wheels.w1_in1, pins.wheels.w1_in2, w1_pwm),
            Tb6612::new(pins.wheels.w2_in1, pins.wheels.w2_in2, w2_pwm),
            Tb6612::new(pins.wheels.w3_in1, pins.wheels.w3_in2, w3_pwm),
            Tb6612::new(pins.wheels.w4_in1, pins.wheels.w4_in2, w4_pwm),
        );
        base.brake();
        base
    };

    let mut parser = Parser::new();
    let mut drdy_prev = false;

    // Communication watchdog: brake motors if no SPI command in 500 ms (real time).
    let mut last_spi_cycle: u32 = DWT::cycle_count();
    const SPI_WATCHDOG_CYCLES: u32 = 500; // milliseconds — compared after conversion
    let mut watchdog_braked = false;
    let mut last_tof_cycle: u32 = DWT::cycle_count();
    const TOF_INTERVAL_MS: f32 = 100.0;
    let mut tof_range_mm: u16 = 0xFFFF; // 0xFFFF = no reading

    let mut last_pid_cycle: u32 = DWT::cycle_count();
    const PID_INTERVAL_MS: f32 = 20.0;

    loop {
        let now = DWT::cycle_count();

        let pid_elapsed_ms = now.wrapping_sub(last_pid_cycle) as f32 / (sysclk_hz / 1000.0);
        if pid_elapsed_ms >= PID_INTERVAL_MS {
            let dt = pid_elapsed_ms / 1000.0;
            m1.step(dt);
            m2.step(dt);
            last_pid_cycle = now;
        }

        let ms_since_spi = now.wrapping_sub(last_spi_cycle) as f32 / (sysclk_hz / 1000.0);
        if ms_since_spi >= SPI_WATCHDOG_CYCLES as f32 && !watchdog_braked {
            writeln!(usart, "WATCHDOG: no SPI in 500ms, braking motors\r").ok();
            m1.mode = LinearMode::Disabled;
            m2.mode = LinearMode::Disabled;
            m1.actuator.brake();
            m2.actuator.brake();
            led_green.off();
            led_yellow.off();
            watchdog_braked = true;
        }

        let tof_elapsed_ms = now.wrapping_sub(last_tof_cycle) as f32 / (sysclk_hz / 1000.0);
        if tof_elapsed_ms >= TOF_INTERVAL_MS {
            last_tof_cycle = now;
            if let Some(ref mut sensor) = tof {
                match sensor.read_range_mm() {
                    Ok(mm) => tof_range_mm = mm,
                    Err(_) => tof_range_mm = 0xFFFF,
                }
            }
        }

        if m1.actuator.is_limit_braking() || m2.actuator.is_limit_braking() {
            led_red.on();
        } else {
            led_red.off();
        }

        let drdy_now = drdy.is_high();
        if drdy_now && !drdy_prev {
            delay.delay_ms(2_u32);

            let mut buf = [0u8; 128];

            let p16_raw = m1.actuator.position_raw();
            let t16_raw = m2.actuator.position_raw();

            let p16_lo = p16_raw as u8;
            let p16_hi = (p16_raw >> 8) as u8;
            let t16_lo = t16_raw as u8;
            let t16_hi = (t16_raw >> 8) as u8;

            let tof_lo = tof_range_mm as u8;
            let tof_hi = (tof_range_mm >> 8) as u8;

            buf[0] = omnitiles::protocol::messages::START_BYTE;
            buf[1] = omnitiles::protocol::messages::MSG_TELEMETRY;
            buf[2] = p16_lo;
            buf[3] = p16_hi;
            buf[4] = t16_lo;
            buf[5] = t16_hi;
            buf[6] = tof_lo;
            buf[7] = tof_hi;
            buf[8] = buf[1]
                .wrapping_add(buf[2])
                .wrapping_add(buf[3])
                .wrapping_add(buf[4])
                .wrapping_add(buf[5])
                .wrapping_add(buf[6])
                .wrapping_add(buf[7]);

            cs1.select();
            delay.delay_us(50_u32);
            spi_bus.transfer_in_place(&mut buf).unwrap_or_default();
            delay.delay_us(50_u32);
            cs1.deselect();
            last_spi_cycle = DWT::cycle_count();
            watchdog_braked = false;

            for &byte in &buf {
                if let Some(cmd) = parser.push(byte) {
                    match cmd {
                        Command::Ping => {
                            writeln!(usart, "cmd: PING — System is alive.\r").ok();
                        }
                        Command::M1Extend(speed) => {
                            writeln!(usart, "cmd: M1Extend speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m1.mode = LinearMode::Disabled;
                            m1.actuator.set_speed(s);
                            led_green.on();
                        }
                        Command::M1Retract(speed) => {
                            writeln!(usart, "cmd: M1Retract speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m1.mode = LinearMode::Disabled;
                            m1.actuator.set_speed(-s);
                            led_green.on();
                        }
                        Command::M1Brake => {
                            writeln!(usart, "cmd: M1Brake\r").ok();
                            m1.mode = LinearMode::Disabled;
                            m1.actuator.brake();
                            led_green.off();
                        }
                        Command::M1SetPosition(mm) => {
                            writeln!(usart, "cmd: M1SetPosition mm={}\r", mm).ok();
                            m1.mode = LinearMode::PositionControl;
                            m1.set_target_position_mm(mm as f32);
                            led_green.on();
                        }
                        Command::M2Extend(speed) => {
                            writeln!(usart, "cmd: M2Extend speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m2.mode = LinearMode::Disabled;
                            m2.actuator.set_speed(s);
                            led_yellow.on();
                        }
                        Command::M2Retract(speed) => {
                            writeln!(usart, "cmd: M2Retract speed={}\r", speed).ok();
                            let s = speed_to_float(speed);
                            m2.mode = LinearMode::Disabled;
                            m2.actuator.set_speed(-s);
                            led_yellow.on();
                        }
                        Command::M2Brake => {
                            writeln!(usart, "cmd: M2Brake\r").ok();
                            m2.mode = LinearMode::Disabled;
                            m2.actuator.brake();
                            led_yellow.off();
                        }
                        Command::M2SetPosition(mm) => {
                            writeln!(usart, "cmd: M2SetPosition mm={}\r", mm).ok();
                            m2.mode = LinearMode::PositionControl;
                            m2.set_target_position_mm(mm as f32);
                            led_yellow.on();
                        }
                        #[cfg(feature = "mobile-base")]
                        Command::BaseVelocity { vx, vy, omega } => {
                            writeln!(
                                usart,
                                "cmd: BaseVelocity vx={} vy={} omega={}\r",
                                vx, vy, omega
                            )
                            .ok();
                            base.set_velocity(
                                vx as f32 / 127.0,
                                vy as f32 / 127.0,
                                omega as f32 / 127.0,
                            );
                        }
                        #[cfg(feature = "mobile-base")]
                        Command::BaseBrake => {
                            writeln!(usart, "cmd: BaseBrake\r").ok();
                            base.brake();
                        }
                        #[cfg(not(feature = "mobile-base"))]
                        _ => {}
                    }
                }
            }
        }
        drdy_prev = drdy_now;
    }
}
