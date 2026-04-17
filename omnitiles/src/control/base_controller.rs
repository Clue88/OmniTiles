// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Four-wheel mecanum base controller.
//!
//! Wraps four `Tb6612` drivers and applies mecanum inverse kinematics to
//! convert body-frame velocity commands into per-wheel speeds.

use crate::control::mecanum;
use crate::drivers::tb6612::Tb6612;

use stm32f7xx_hal::prelude::*;

pub struct BaseController<
    const W1_IN1_P: char,
    const W1_IN1_N: u8,
    const W1_IN2_P: char,
    const W1_IN2_N: u8,
    W1Pwm,
    const W2_IN1_P: char,
    const W2_IN1_N: u8,
    const W2_IN2_P: char,
    const W2_IN2_N: u8,
    W2Pwm,
    const W3_IN1_P: char,
    const W3_IN1_N: u8,
    const W3_IN2_P: char,
    const W3_IN2_N: u8,
    W3Pwm,
    const W4_IN1_P: char,
    const W4_IN1_N: u8,
    const W4_IN2_P: char,
    const W4_IN2_N: u8,
    W4Pwm,
> {
    pub fl: Tb6612<W1_IN1_P, W1_IN1_N, W1_IN2_P, W1_IN2_N, W1Pwm>,
    pub fr: Tb6612<W2_IN1_P, W2_IN1_N, W2_IN2_P, W2_IN2_N, W2Pwm>,
    pub rl: Tb6612<W3_IN1_P, W3_IN1_N, W3_IN2_P, W3_IN2_N, W3Pwm>,
    pub rr: Tb6612<W4_IN1_P, W4_IN1_N, W4_IN2_P, W4_IN2_N, W4Pwm>,
}

impl<
        const W1_IN1_P: char,
        const W1_IN1_N: u8,
        const W1_IN2_P: char,
        const W1_IN2_N: u8,
        W1Pwm: _embedded_hal_PwmPin<Duty = u16>,
        const W2_IN1_P: char,
        const W2_IN1_N: u8,
        const W2_IN2_P: char,
        const W2_IN2_N: u8,
        W2Pwm: _embedded_hal_PwmPin<Duty = u16>,
        const W3_IN1_P: char,
        const W3_IN1_N: u8,
        const W3_IN2_P: char,
        const W3_IN2_N: u8,
        W3Pwm: _embedded_hal_PwmPin<Duty = u16>,
        const W4_IN1_P: char,
        const W4_IN1_N: u8,
        const W4_IN2_P: char,
        const W4_IN2_N: u8,
        W4Pwm: _embedded_hal_PwmPin<Duty = u16>,
    >
    BaseController<
        W1_IN1_P,
        W1_IN1_N,
        W1_IN2_P,
        W1_IN2_N,
        W1Pwm,
        W2_IN1_P,
        W2_IN1_N,
        W2_IN2_P,
        W2_IN2_N,
        W2Pwm,
        W3_IN1_P,
        W3_IN1_N,
        W3_IN2_P,
        W3_IN2_N,
        W3Pwm,
        W4_IN1_P,
        W4_IN1_N,
        W4_IN2_P,
        W4_IN2_N,
        W4Pwm,
    >
{
    pub fn new(
        fl: Tb6612<W1_IN1_P, W1_IN1_N, W1_IN2_P, W1_IN2_N, W1Pwm>,
        fr: Tb6612<W2_IN1_P, W2_IN1_N, W2_IN2_P, W2_IN2_N, W2Pwm>,
        rl: Tb6612<W3_IN1_P, W3_IN1_N, W3_IN2_P, W3_IN2_N, W3Pwm>,
        rr: Tb6612<W4_IN1_P, W4_IN1_N, W4_IN2_P, W4_IN2_N, W4Pwm>,
    ) -> Self {
        Self { fl, fr, rl, rr }
    }

    /// Set body-frame velocity. Inputs are normalized: -1.0..1.0 for each axis.
    pub fn set_velocity(&mut self, vx: f32, vy: f32, omega: f32) {
        let [fl, fr, rl, rr] = mecanum::mecanum_ik(vx, vy, omega);
        self.fl.set_speed(-fl);
        self.fr.set_speed(fr);
        self.rl.set_speed(-rl);
        self.rr.set_speed(rr);
    }

    /// Brake all four wheels.
    pub fn brake(&mut self) {
        self.fl.brake();
        self.fr.brake();
        self.rl.brake();
        self.rr.brake();
    }
}
