// SPDX-License-Identifier: MIT
// (c) 2025-2026 Christopher Liu

//! PID position control for Actuonix linear actuators.

use crate::control::Pid;
use crate::drivers::ActuonixLinear;
use stm32f7xx_hal::prelude::*;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum LinearMode {
    PositionControl,
    Disabled,
}

/// PID position controller for an Actuonix linear actuator. Call [`step`](Self::step) periodically.
pub struct LinearController<
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
    pub actuator: ActuonixLinear<CS_P, CS_N, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos>,
    pub pid: Pid,
    pub mode: LinearMode,

    pub target_position_mm: f32,
    pub min_position_mm: f32,
    pub max_position_mm: f32,
    pub on_target_tolerance_mm: f32,
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
    > LinearController<CS_P, CS_N, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos>
where
    Pwm1: _embedded_hal_PwmPin<Duty = u16>,
    Pwm2: _embedded_hal_PwmPin<Duty = u16>,
    ReadPos: FnMut() -> u16,
{
    /// Create a new linear controller with PID gains and limits.
    pub fn new(
        actuator: ActuonixLinear<CS_P, CS_N, SLP_P, SLP_N, DIS_P, DIS_N, Pwm1, Pwm2, ReadPos>,
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

    /// Run one control step.
    pub fn step(&mut self, dt: f32) {
        self.actuator.enforce_limits();

        match self.mode {
            LinearMode::Disabled => return,

            LinearMode::PositionControl => {
                let position_mm = self.actuator.position_mm();
                let target = self
                    .target_position_mm
                    .clamp(self.min_position_mm, self.max_position_mm);
                let error = target - position_mm;

                if error.abs() <= self.on_target_tolerance_mm {
                    self.actuator.brake();
                    return;
                }

                let output = self.pid.update(target, position_mm, dt);
                self.actuator.set_speed(output);
            }
        }
    }
}
