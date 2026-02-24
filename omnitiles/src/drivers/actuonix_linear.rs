// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Generic driver for Actuonix 16-series (P16, T16) Linear Actuators with potentiometer control.
//!
//! Wraps a DRV8873 H-Bridge for motor control and an ADC channel for position feedback.
//!
//! Wiring (Option -P):
//! - Pin 1 (Orange): Potentiometer Ground
//! - Pin 2 (Purple): Potentiometer Wiper (ADC Input)
//! - Pin 3 (Red):    Motor Terminal A (+)
//! - Pin 4 (Black):  Motor Terminal B (-)
//! - Pin 5 (Yellow): Potentiometer Reference (3.3V)

use crate::drivers::drv8873::{Drv8873, Fault};
use crate::hw::SpiBus;

use stm32f7xx_hal::{
    gpio::{self, Output, PushPull},
    prelude::*,
    spi,
};

/// Logical drive direction for the linear actuator.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Direction {
    Extend,
    Retract,
    Brake,
}

/// Generic driver for Actuonix linear actuators (P16, T16).
///
/// `ReadPos` is a closure that returns the raw 12-bit ADC reading (0..4095).
pub struct ActuonixLinear<
    const CS_P: char,
    const CS_N: u8,
    const SLP_P: char,
    const SLP_N: u8,
    const DIS_P: char,
    const DIS_N: u8,
    Pwm1,
    Pwm2,
    ReadPos,
> {
    drv: Drv8873<CS_P, CS_N>,
    pwm1: Pwm1,
    pwm2: Pwm2,
    nsleep: gpio::Pin<SLP_P, SLP_N, Output<PushPull>>,
    disable: gpio::Pin<DIS_P, DIS_N, Output<PushPull>>,
    read_position: ReadPos,
    adc_history: [u16; 5],
    adc_idx: usize,
    ema_pos: f32,
    stroke_len_mm: f32,
    buffer_mm: f32,
    current_speed: f32,
}

impl<
        const CS_P: char,
        const CS_N: u8,
        const SLP_P: char,
        const SLP_N: u8,
        const DIS_P: char,
        const DIS_N: u8,
        Pwm1,
        Pwm2,
        ReadPos,
    > ActuonixLinear<CS_P, CS_N, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos>
where
    Pwm1: _embedded_hal_PwmPin<Duty = u16>,
    Pwm2: _embedded_hal_PwmPin<Duty = u16>,
    ReadPos: FnMut() -> u16,
{
    /// Construct a new Actuonix driver with Hardware PWM.
    pub fn new<SlpMode, DisMode>(
        drv: Drv8873<CS_P, CS_N>,
        pwm1: Pwm1,
        pwm2: Pwm2,
        nsleep: gpio::Pin<SLP_P, SLP_N, SlpMode>,
        disable: gpio::Pin<DIS_P, DIS_N, DisMode>,
        mut read_position: ReadPos,
        stroke_len_mm: f32,
        buffer_mm: f32,
    ) -> Self {
        let mut nsleep = nsleep.into_push_pull_output();
        let mut disable = disable.into_push_pull_output();

        // Default: Awake, Enabled
        nsleep.set_high();
        disable.set_low();

        let initial_pos = (read_position)();

        Self {
            drv,
            pwm1,
            pwm2,
            nsleep,
            disable,
            read_position,
            adc_history: [initial_pos; 5],
            adc_idx: 0,
            ema_pos: initial_pos as f32,
            stroke_len_mm,
            buffer_mm,
            current_speed: 0.0,
        }
    }

    /// Set the motor speed and direction.
    ///
    /// `speed` - A float from -1.0 (Full Retract) to 1.0 (Full Extend).
    pub fn set_speed(&mut self, speed: f32) {
        // Clamp speed to valid range
        let mut speed = speed.clamp(-1.0, 1.0);

        let pos = self.position_mm();
        let max_pos = self.stroke_len_mm - self.buffer_mm;
        let min_pos = self.buffer_mm;

        // Prevent starting a movement that goes deeper into the out-of-bounds area
        if speed > 0.0 && pos >= max_pos {
            speed = 0.0;
        } else if speed < 0.0 && pos <= min_pos {
            speed = 0.0;
        }

        self.current_speed = speed;

        let max_duty = self.pwm1.get_max_duty(); // Assuming Pwm1/Pwm2 have same resolution

        // Calculate target duty cycle
        let duty = (speed.abs() * max_duty as f32) as u16;

        if speed > 0.001 {
            // Extend: IN1 PWM, IN2 Low
            self.pwm1.set_duty(duty);
            self.pwm2.set_duty(0);
            self.pwm1.enable();
            self.pwm2.enable();
        } else if speed < -0.001 {
            // Retract: IN1 Low, IN2 PWM
            self.pwm1.set_duty(0);
            self.pwm2.set_duty(duty);
            self.pwm1.enable();
            self.pwm2.enable();
        } else {
            self.brake();
        }
    }

    /// Continuously check the ADC position and brake if the actuator exceeds the software limits.
    /// This should be called regularly in the main application loop.
    pub fn enforce_limits(&mut self) {
        // If we aren't moving, no need to check
        if self.current_speed.abs() < 0.001 {
            return;
        }

        let pos = self.position_mm();
        let max_pos = self.stroke_len_mm - self.buffer_mm;
        let min_pos = self.buffer_mm;

        if self.current_speed > 0.0 && pos >= max_pos {
            self.brake();
            self.current_speed = 0.0; // Reset state
        } else if self.current_speed < 0.0 && pos <= min_pos {
            self.brake();
            self.current_speed = 0.0; // Reset state
        }
    }

    /// Extend the actuator at full speed.
    #[inline]
    pub fn extend(&mut self) {
        self.set_speed(1.0);
    }

    /// Retract the actuator at full speed.
    #[inline]
    pub fn retract(&mut self) {
        self.set_speed(-1.0);
    }

    /// Brake (stops quickly by shorting motor terminals).
    #[inline]
    pub fn brake(&mut self) {
        let max = self.pwm1.get_max_duty();
        self.pwm1.set_duty(max);
        self.pwm2.set_duty(max);
        self.pwm1.enable();
        self.pwm2.enable();
    }

    /// Read raw 12-bit ADC value (0-4095) with a median + EMA low-pass filter.
    #[inline]
    pub fn position_raw(&mut self) -> u16 {
        // 1. Get fresh reading and update median ring buffer
        let raw = (self.read_position)();

        self.adc_history[self.adc_idx] = raw;
        self.adc_idx = (self.adc_idx + 1) % 5;

        // 2. Calculate Median (kills sharp, single-read spikes)
        let mut sorted = self.adc_history;
        sorted.sort_unstable();
        let median = sorted[2] as f32;

        // 3. Calculate EMA (smooths out sustained high-frequency jitter)
        // Uses 98% of the old value and 2% of the new value.
        self.ema_pos = (self.ema_pos * 0.98) + (median * 0.02);

        self.ema_pos as u16
    }

    /// Read position as a fraction (0.0 = Retracted, 1.0 = Extended).
    pub fn position_percent(&mut self) -> f32 {
        let raw = self.position_raw();
        // 12-bit ADC = 4095 max.
        (raw as f32) / 4095.0
    }

    /// Read position in millimeters.
    pub fn position_mm(&mut self) -> f32 {
        self.position_percent() * self.stroke_len_mm
    }

    /// Get the max stroke length.
    pub fn stroke_len_mm(&self) -> f32 {
        self.stroke_len_mm
    }

    /// Access the inner DRV8873 for fault reading.
    pub fn drv(&mut self) -> &mut Drv8873<CS_P, CS_N> {
        &mut self.drv
    }

    /// Put the driver into sleep mode.
    ///
    /// This shuts down most of the internal circuitry to reduce power consumption.
    #[inline]
    pub fn sleep(&mut self) {
        self.nsleep.set_low();
    }

    /// Wake the driver from sleep mode.
    #[inline]
    pub fn wake(&mut self) {
        self.nsleep.set_high();
    }

    /// Enable the motor and wake the driver if in sleep.
    #[inline]
    pub fn enable_outputs(&mut self) {
        self.wake();
        self.disable.set_low();
    }

    /// Disable the motor and brake.
    #[inline]
    pub fn disable_outputs(&mut self) {
        self.brake();
        self.disable.set_high();
    }

    /// Read the FAULT status register from the DRV8873.
    pub fn read_fault<I, PINS>(
        &mut self,
        spi_bus: &mut SpiBus<I, PINS>,
    ) -> Result<Fault, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        self.drv.read_fault(spi_bus)
    }
}
