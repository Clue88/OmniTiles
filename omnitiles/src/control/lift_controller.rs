//! Closed-loop controller for the linear lift actuator.
//!
//! This controller wraps a [`LiftMotor`] and provides a periodic `step()` function that computes a
//! drive command using PID control and applies it to the motor.
//!
//! Typical usage pattern:
//!
//! ```no_run
//! controller.set_target_height_mm(150.0);
//!
//! loop {
//!     controller.step(dt_seconds);
//!     delay.delay_ms(10_u32);
//! }
//! ```

use crate::control::Pid;
use crate::motors::LiftMotor;

/// Operating mode of the lift controller.
#[derive(Copy, Clone, Debug)]
pub enum LiftMode {
    /// Regular closed-loop control toward a height target.
    PositionControl,

    /// Motor is manually disabled (coast).
    Disabled,
}

/// Controller state and configuration.
pub struct LiftController<
    const CS_P: char,
    const CS_N: u8,
    const IN1_P: char,
    const IN1_N: u8,
    const IN2_P: char,
    const IN2_N: u8,
    const SLP_P: char,
    const SLP_N: u8,
    const DIS_P: char,
    const DIS_N: u8,
> {
    motor: LiftMotor<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>,
    pid: Pid,
    mode: LiftMode,

    /// Commanded height (mm)
    target_height_mm: f32,

    /// Safety limits (mm)
    min_height_mm: f32,
    max_height_mm: f32,

    /// Deadband in mm, "close enough" to target
    on_target_tolerance_mm: f32,
}

impl<
        const CS_P: char,
        const CS_N: u8,
        const IN1_P: char,
        const IN1_N: u8,
        const IN2_P: char,
        const IN2_N: u8,
        const SLP_P: char,
        const SLP_N: u8,
        const DIS_P: char,
        const DIS_N: u8,
    > LiftController<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>
{
    /// Create a new lift controller with PID gains and limits.
    pub fn new(
        motor: LiftMotor<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>,
        pid: Pid,
        min_height_mm: f32,
        max_height_mm: f32,
        on_target_tolerance_mm: f32,
    ) -> Self {
        Self {
            motor,
            pid,
            mode: LiftMode::PositionControl,
            target_height_mm: 0.0,
            min_height_mm,
            max_height_mm,
            on_target_tolerance_mm,
        }
    }

    /// Set a new target height (mm), automatically clamped to limits.
    pub fn set_target_height_mm(&mut self, mm: f32) {
        let clamped = mm.clamp(self.min_height_mm, self.max_height_mm);
        self.target_height_mm = clamped;
        self.pid.reset();
    }

    /// Directly disable the lift (coast).
    pub fn disable(&mut self) {
        self.mode = LiftMode::Disabled;
        self.motor.inner_motor().coast();
    }

    /// Re-enable position control.
    pub fn enable(&mut self) {
        self.mode = LiftMode::PositionControl;
    }

    /// Current measured height (mm).
    #[inline]
    pub fn height_mm(&self) -> f32 {
        self.motor.height_mm()
    }

    /// Returns true if the lift is within tolerance of its commanded height.
    pub fn on_target(&self) -> bool {
        let err = (self.height_mm() - self.target_height_mm).abs();
        err <= self.on_target_tolerance_mm
    }

    /// Run one control step.
    ///
    /// `dt` — time delta in seconds since the previous call, e.g. 0.01 for 10ms.
    pub fn step(&mut self, dt: f32) {
        match self.mode {
            LiftMode::Disabled => {
                self.motor.inner_motor().coast();
                return;
            }

            LiftMode::PositionControl => {
                let current = self.motor.height_mm();
                let target = self.target_height_mm;
                let error = target - current;

                // PID output
                let mut u = self.pid.update(target, current, dt);

                // Clamp to [-1.0, 1.0]
                if u > 1.0 {
                    u = 1.0;
                }
                if u < -1.0 {
                    u = -1.0;
                }

                // Near-target deadband
                if error.abs() <= self.on_target_tolerance_mm {
                    self.motor.inner_motor().brake();
                    return;
                }

                // Apply PID output
                self.motor.apply_pid_output(u);
            }
        }
    }
}
