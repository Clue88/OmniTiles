//! Linear actuator interface built on top of `SpiMotor`.
//!
//! This module converts encoder ticks into physical height and provides target-based movement
//! helpers, which can be used in a higher-level controller like a PID loop.

use crate::drivers::Fit0185;

/// Lift motor abstraction layered on top of a `SpiMotor`.
///
/// Geometry parameters:
/// - `mm_per_rev`: mechanical displacement per output shaft revolution
/// - `counts_per_rev`: inherited automatically from underlying motor
pub struct LiftMotor<
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
    motor: Fit0185<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>,
    mm_per_rev: f32,
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
    > LiftMotor<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>
{
    /// Create a new `LiftMotor` wrapper.
    ///
    /// `mm_per_rev` — how many millimeters the actuator moves per full revolution.
    pub fn new(
        motor: Fit0185<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>,
        mm_per_rev: f32,
    ) -> Self {
        Self { motor, mm_per_rev }
    }

    /// Convert encoder ticks to millimeters of extension.
    pub fn height_mm(&self) -> f32 {
        let revs = self.motor.position_revs();
        revs * self.mm_per_rev
    }

    /// Convert from mm to encoder ticks.
    fn ticks_for_mm(&self, mm: f32) -> i32 {
        let revs = mm / self.mm_per_rev;
        self.motor.ticks_for_revs(revs)
    }

    /// Compute the target tick position for an absolute height command.
    pub fn target_ticks_for_height(&self, mm: f32) -> i32 {
        let revs = mm / self.mm_per_rev;
        self.motor.ticks_for_revs(revs)
    }

    /// Compute a relative tick movement for a delta height.
    pub fn delta_ticks_for_height(&self, delta_mm: f32) -> i32 {
        self.ticks_for_mm(delta_mm)
    }

    /// Move by a relative number of millimeters.
    pub fn move_by_mm(&mut self, delta_mm: f32) -> i32 {
        let ticks = self.delta_ticks_for_height(delta_mm);
        self.motor.target_for_delta_ticks(ticks)
    }

    /// Move by a relative number of revolutions.
    pub fn move_by_revs(&mut self, revs: f32) -> i32 {
        let ticks = self.motor.ticks_for_revs(revs);
        self.motor.target_for_delta_ticks(ticks)
    }

    /// Command the lift to go to a specific height (mm), returning a tick target.
    pub fn go_to_height_mm(&mut self, mm: f32) -> i32 {
        let target = self.target_ticks_for_height(mm);
        target
    }

    /// Reset encoder reference (zero height).
    pub fn zero(&mut self) {
        self.motor.zero();
    }

    /// Expose the underlying `SpiMotor`.
    pub fn inner_motor(
        &mut self,
    ) -> &mut Fit0185<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N> {
        &mut self.motor
    }

    /// Apply PID output to the underlying motor.
    pub fn apply_pid_output(&mut self, u: f32) {
        self.motor.apply_pid_output(u)
    }
}
