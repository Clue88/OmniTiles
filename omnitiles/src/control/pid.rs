// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Generic PID controller for closed-loop control.
//!
//! Works in `no_std` and does not allocate memory.

/// PID controller with tunable gains and output clamping.
pub struct Pid {
    /// Proportional gain
    kp: f32,
    /// Integral gain
    ki: f32,
    /// Derivative gain
    kd: f32,

    /// Integrator state
    integral: f32,
    /// Last process variable (for derivative term)
    prev_measurement: f32,

    /// Output clamp
    out_min: f32,
    out_max: f32,

    /// Integral anti-windup clamp
    int_min: f32,
    int_max: f32,

    first_update: bool,
}

impl Pid {
    /// Create a new PID controller.
    ///
    /// `kp`, `ki`, `kd` are the gain constants.
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,

            integral: 0.0,
            prev_measurement: 0.0,

            out_min: -1.0,
            out_max: 1.0,

            int_min: -1.0,
            int_max: 1.0,

            first_update: true,
        }
    }

    /// Set output limits.
    pub fn with_output_limits(mut self, min: f32, max: f32) -> Self {
        self.out_min = min;
        self.out_max = max;
        self
    }

    /// Set integral limits for anti-windup.
    pub fn with_integral_limits(mut self, min: f32, max: f32) -> Self {
        self.int_min = min;
        self.int_max = max;
        self
    }

    /// Reset integrator + derivative history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.first_update = true;
    }

    /// Update the controller.
    ///
    /// `setpoint` — desired value  
    /// `measurement` — current value  
    /// `dt` — timestep in seconds (e.g. 0.02 for 50 Hz control loop)
    ///
    /// Returns a normalized command in [`out_min`, `out_max`] which can be mapped to motor drive.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        // ----- P term -----
        let p = self.kp * error;

        // ----- I term -----
        self.integral += error * dt * self.ki;

        // Anti-windup clamp
        if self.integral > self.int_max {
            self.integral = self.int_max;
        }
        if self.integral < self.int_min {
            self.integral = self.int_min;
        }

        let i = self.integral;

        // ----- D term (on measurement to reduce noise sensitivity) -----
        let d = if self.first_update {
            self.first_update = false;
            0.0
        } else {
            let dv = self.prev_measurement - measurement;
            self.kd * (dv / dt)
        };
        self.prev_measurement = measurement;

        // ----- Output clamp -----
        let mut out = p + i + d;
        if out > self.out_max {
            out = self.out_max;
        }
        if out < self.out_min {
            out = self.out_min;
        }

        out
    }
}
