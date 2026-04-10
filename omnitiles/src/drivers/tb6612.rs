// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Single-channel driver for the TB6612FNG H-bridge (used on DFRobot DRI0040).
//!
//! Each TB6612 channel controls one DC motor via:
//! - One PWM pin for speed (duty cycle)
//! - Two GPIO pins (IN1/IN2) for direction

use stm32f7xx_hal::{
    gpio::{self, Output, PushPull},
    prelude::*,
};

pub struct Tb6612<const IN1_P: char, const IN1_N: u8, const IN2_P: char, const IN2_N: u8, Pwm> {
    in1: gpio::Pin<IN1_P, IN1_N, Output<PushPull>>,
    in2: gpio::Pin<IN2_P, IN2_N, Output<PushPull>>,
    pwm: Pwm,
}

impl<const IN1_P: char, const IN1_N: u8, const IN2_P: char, const IN2_N: u8, Pwm>
    Tb6612<IN1_P, IN1_N, IN2_P, IN2_N, Pwm>
where
    Pwm: _embedded_hal_PwmPin<Duty = u16>,
{
    pub fn new(
        in1: gpio::Pin<IN1_P, IN1_N, Output<PushPull>>,
        in2: gpio::Pin<IN2_P, IN2_N, Output<PushPull>>,
        mut pwm: Pwm,
    ) -> Self {
        pwm.set_duty(0);
        pwm.enable();
        Self { in1, in2, pwm }
    }

    /// Set motor speed from -1.0 (full reverse) to 1.0 (full forward).
    /// Values near zero (|speed| < 0.001) engage the brake.
    pub fn set_speed(&mut self, speed: f32) {
        let speed = speed.clamp(-1.0, 1.0);

        if speed > 0.001 {
            self.in1.set_high();
            self.in2.set_low();
            let duty = (speed * self.pwm.get_max_duty() as f32) as u16;
            self.pwm.set_duty(duty);
        } else if speed < -0.001 {
            self.in1.set_low();
            self.in2.set_high();
            let duty = (speed.abs() * self.pwm.get_max_duty() as f32) as u16;
            self.pwm.set_duty(duty);
        } else {
            self.brake();
        }
    }

    /// Short-brake: both inputs high, full duty.
    pub fn brake(&mut self) {
        self.in1.set_high();
        self.in2.set_high();
        self.pwm.set_duty(self.pwm.get_max_duty());
    }

    /// Coast: both inputs low, zero duty.
    pub fn coast(&mut self) {
        self.in1.set_low();
        self.in2.set_low();
        self.pwm.set_duty(0);
    }
}
