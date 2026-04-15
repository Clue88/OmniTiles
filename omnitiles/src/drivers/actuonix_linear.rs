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
use crate::hw::spi::CsControl;
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
/// Supports ganging `N` physical actuators driven in parallel by the same
/// H-bridge but each with their own potentiometer channel. Channels marked
/// `inverted` have their raw reading mirrored (`4095 - raw`) before fusion,
/// which undoes swapped wiper wiring on mechanically opposed units. The
/// per-channel `enabled` flags select which pots contribute to the fused
/// position estimate; disabled channels are still sampled so telemetry can
/// see them, they just don't influence control.
///
/// `ReadPos` is a closure that returns raw 12-bit ADC readings (0..4095) for
/// all `N` channels in one call.
pub struct ActuonixLinear<
    CS: CsControl,
    const SLP_P: char,
    const SLP_N: u8,
    const DIS_P: char,
    const DIS_N: u8,
    Pwm1,
    Pwm2,
    ReadPos,
    const N: usize,
> {
    drv: Drv8873<CS>,
    pwm1: Pwm1,
    pwm2: Pwm2,
    nsleep: gpio::Pin<SLP_P, SLP_N, Output<PushPull>>,
    disable: gpio::Pin<DIS_P, DIS_N, Output<PushPull>>,
    read_positions: ReadPos,
    adc_history: [[u16; N]; 5],
    adc_idx: usize,
    last_medians: [u16; N],
    inverted: [bool; N],
    enabled: [bool; N],
    stroke_len_mm: f32,
    inverted_pair_sum_mm: f32,
    buffer_bottom_mm: f32,
    buffer_top_mm: f32,
    current_speed: f32,
    limit_brake_active: bool,
}

impl<
        CS: CsControl,
        const SLP_P: char,
        const SLP_N: u8,
        const DIS_P: char,
        const DIS_N: u8,
        Pwm1,
        Pwm2,
        ReadPos,
        const N: usize,
    > ActuonixLinear<CS, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos, N>
where
    Pwm1: _embedded_hal_PwmPin<Duty = u16>,
    Pwm2: _embedded_hal_PwmPin<Duty = u16>,
    ReadPos: FnMut() -> [u16; N],
{
    /// Construct a new Actuonix driver with Hardware PWM.
    ///
    /// `inverted[i]` should be true for channels whose pot wiper is wired
    /// inversely to the extension direction (e.g., mechanically opposed units
    /// that share the same drive signal). All channels start enabled.
    pub fn new<SlpMode, DisMode>(
        drv: Drv8873<CS>,
        pwm1: Pwm1,
        pwm2: Pwm2,
        nsleep: gpio::Pin<SLP_P, SLP_N, SlpMode>,
        disable: gpio::Pin<DIS_P, DIS_N, DisMode>,
        mut read_positions: ReadPos,
        inverted: [bool; N],
        stroke_len_mm: f32,
        inverted_pair_sum_mm: f32,
        buffer_bottom_mm: f32,
        buffer_top_mm: f32,
    ) -> Self {
        let mut nsleep = nsleep.into_push_pull_output();
        let mut disable = disable.into_push_pull_output();

        // Default: Awake, Enabled
        nsleep.set_high();
        disable.set_low();

        let initial = (read_positions)();

        Self {
            drv,
            pwm1,
            pwm2,
            nsleep,
            disable,
            read_positions,
            adc_history: [initial; 5],
            adc_idx: 0,
            last_medians: initial,
            inverted,
            enabled: [true; N],
            stroke_len_mm,
            inverted_pair_sum_mm,
            buffer_bottom_mm,
            buffer_top_mm,
            current_speed: 0.0,
            limit_brake_active: false,
        }
    }

    /// Enable or disable a specific potentiometer channel. Disabled channels
    /// are still sampled for telemetry but do not contribute to the fused
    /// position estimate used by control.
    pub fn set_channel_enabled(&mut self, channel: usize, enabled: bool) {
        if channel < N {
            self.enabled[channel] = enabled;
        }
    }

    /// True if at least one potentiometer channel is enabled for fusion.
    #[inline]
    pub fn any_channel_enabled(&self) -> bool {
        self.enabled.iter().any(|e| *e)
    }

    /// Access the last computed per-channel medians (raw, uninverted). Useful
    /// for telemetry. Returns the values from the most recent refresh.
    #[inline]
    pub fn channel_medians(&self) -> &[u16; N] {
        &self.last_medians
    }

    /// Sample all channels once, update the per-channel median filter, and
    /// cache the new medians. Called by [`position_raw`](Self::position_raw).
    fn refresh(&mut self) {
        let raw = (self.read_positions)();
        self.adc_history[self.adc_idx] = raw;
        self.adc_idx = (self.adc_idx + 1) % 5;

        for i in 0..N {
            let mut column = [0u16; 5];
            for r in 0..5 {
                column[r] = self.adc_history[r][i];
            }
            column.sort_unstable();
            self.last_medians[i] = column[2];
        }
    }

    /// Compute the fused raw position from the current cached medians. Returns
    /// `None` if no channels are enabled.
    fn fused_raw_from_cache(&self) -> Option<u16> {
        let mut sum: u32 = 0;
        let mut count: u32 = 0;
        for i in 0..N {
            if self.enabled[i] {
                let m = self.last_medians[i] as u32;
                let logical = if self.inverted[i] {
                    let offset = (self.inverted_pair_sum_mm / self.stroke_len_mm * 4095.0) as u32;
                    offset.saturating_sub(m)
                } else {
                    m
                };
                sum += logical;
                count += 1;
            }
        }
        if count == 0 {
            None
        } else {
            Some((sum / count) as u16)
        }
    }

    /// Set the motor speed and direction.
    ///
    /// `speed` - A float from -1.0 (Full Retract) to 1.0 (Full Extend).
    ///
    /// If no pot channels are enabled, soft-limit checks are skipped so
    /// manual drive still works without position feedback.
    pub fn set_speed(&mut self, speed: f32) {
        // Any explicit speed command clears "limit brake" state.
        self.limit_brake_active = false;

        // Clamp speed to valid range
        let mut speed = speed.clamp(-1.0, 1.0);

        if let Some(pos) = self.position_mm() {
            let max_pos = self.stroke_len_mm - self.buffer_top_mm;
            let min_pos = self.buffer_bottom_mm;

            // Prevent starting a movement that goes deeper into the out-of-bounds area
            if speed > 0.0 && pos >= max_pos {
                speed = 0.0;
            } else if speed < 0.0 && pos <= min_pos {
                speed = 0.0;
            }
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
    /// This should be called regularly in the main application loop. No-op when no
    /// channels are enabled (manual drive without feedback).
    pub fn enforce_limits(&mut self) {
        if self.current_speed.abs() < 0.001 {
            return;
        }

        let Some(pos) = self.position_mm() else {
            return;
        };
        let max_pos = self.stroke_len_mm - self.buffer_top_mm;
        let min_pos = self.buffer_bottom_mm;

        if self.current_speed > 0.0 && pos >= max_pos {
            self.brake_due_to_limit();
            self.current_speed = 0.0;
        } else if self.current_speed < 0.0 && pos <= min_pos {
            self.brake_due_to_limit();
            self.current_speed = 0.0;
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
        self.limit_brake_active = false;
        self.brake_raw();
    }

    #[inline]
    fn brake_due_to_limit(&mut self) {
        self.limit_brake_active = true;
        self.brake_raw();
    }

    #[inline]
    fn brake_raw(&mut self) {
        let max = self.pwm1.get_max_duty();
        self.pwm1.set_duty(max);
        self.pwm2.set_duty(max);
        self.pwm1.enable();
        self.pwm2.enable();
    }

    /// True when we are currently braking due to software limit enforcement.
    #[inline]
    pub fn is_limit_braking(&self) -> bool {
        self.limit_brake_active
    }

    /// Refresh all channels and return the fused raw 12-bit position
    /// (0..4095). Returns `None` if no channels are enabled.
    #[inline]
    pub fn position_raw(&mut self) -> Option<u16> {
        self.refresh();
        self.fused_raw_from_cache()
    }

    /// Read position as a fraction (0.0 = Retracted, 1.0 = Extended).
    pub fn position_percent(&mut self) -> Option<f32> {
        self.position_raw().map(|r| (r as f32) / 4095.0)
    }

    /// Read position in millimeters.
    pub fn position_mm(&mut self) -> Option<f32> {
        self.position_percent().map(|p| p * self.stroke_len_mm)
    }

    /// Get the max stroke length.
    pub fn stroke_len_mm(&self) -> f32 {
        self.stroke_len_mm
    }

    /// Access the inner DRV8873 for fault reading.
    pub fn drv(&mut self) -> &mut Drv8873<CS> {
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
