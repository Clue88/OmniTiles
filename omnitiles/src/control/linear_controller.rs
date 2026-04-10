// SPDX-License-Identifier: MIT
// © 2025-2026 Christopher Liu

//! PID position control for Actuonix linear actuators.

use crate::control::Pid;
use crate::drivers::ActuonixLinear;
use crate::hw::spi::CsControl;
use stm32f7xx_hal::prelude::*;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum LinearMode {
    PositionControl,
    Disabled,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControlError {
    /// PID was requested but no pot channels are enabled on the actuator.
    NoPositionFeedback,
}

/// PID position controller for an Actuonix linear actuator. Call [`step`](Self::step) periodically.
pub struct LinearController<
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
    pub actuator: ActuonixLinear<CS, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos, N>,
    pub pid: Pid,
    pub mode: LinearMode,

    pub target_position_mm: f32,
    pub min_position_mm: f32,
    pub max_position_mm: f32,
    pub on_target_tolerance_mm: f32,
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
    > LinearController<CS, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos, N>
where
    Pwm1: _embedded_hal_PwmPin<Duty = u16>,
    Pwm2: _embedded_hal_PwmPin<Duty = u16>,
    ReadPos: FnMut() -> [u16; N],
{
    /// Create a new linear controller with PID gains and limits.
    pub fn new(
        actuator: ActuonixLinear<CS, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos, N>,
        pid: Pid,
        min_position_mm: f32,
        max_position_mm: f32,
        on_target_tolerance_mm: f32,
    ) -> Self {
        Self {
            actuator,
            pid,
            mode: LinearMode::PositionControl,
            target_position_mm: 0.0,
            min_position_mm,
            max_position_mm,
            on_target_tolerance_mm,
        }
    }

    /// Set a new target position (mm), automatically clamped to limits.
    pub fn set_target_position_mm(&mut self, mm: f32) {
        self.target_position_mm = mm.clamp(self.min_position_mm, self.max_position_mm);
        self.pid.reset();
    }

    /// Run one control step. Returns `Err(NoPositionFeedback)` if the mode is
    /// `PositionControl` but the actuator has no enabled pot channels; in that
    /// case the actuator is braked for safety.
    pub fn step(&mut self, dt: f32) -> Result<(), ControlError> {
        self.actuator.enforce_limits();

        match self.mode {
            LinearMode::Disabled => Ok(()),

            LinearMode::PositionControl => {
                let Some(position_mm) = self.actuator.position_mm() else {
                    self.actuator.brake();
                    return Err(ControlError::NoPositionFeedback);
                };
                let target = self
                    .target_position_mm
                    .clamp(self.min_position_mm, self.max_position_mm);
                let error = target - position_mm;

                if error.abs() <= self.on_target_tolerance_mm {
                    self.actuator.brake();
                    return Ok(());
                }

                let output = self.pid.update(target, position_mm, dt);
                self.actuator.set_speed(output);
                Ok(())
            }
        }
    }
}
