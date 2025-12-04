// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Tilt actuator interface built on top of `CanMotor`.
//!
//! This module converts between the motor's internal shaft angle representation and a physical tile
//! tilt angle, using a configurable gear ratio.

use crate::drivers::Gim6010;

/// High-level tilt motor wrapped around a `CanMotor`.
///
/// Geometry parameters:
/// - `gear_ratio` — number of motor revolutions per tile revolution.
pub struct TiltMotor<const DEV_ADDR: u16> {
    motor: Gim6010<DEV_ADDR>,
    gear_ratio: f32,
}

impl<const DEV_ADDR: u16> TiltMotor<DEV_ADDR> {
    /// Create a new `TiltMotor` from an underlying `CanMotor` and a gear ratio.
    pub fn new(motor: Gim6010<DEV_ADDR>, gear_ratio: f32) -> Self {
        Self { motor, gear_ratio }
    }

    /// Access the underlying `CanMotor`.
    #[inline]
    pub fn inner_motor(&mut self) -> &mut Gim6010<DEV_ADDR> {
        &mut self.motor
    }

    /// Convert a raw encoder value [0..65535] into motor shaft angle in radians.
    #[inline]
    pub fn shaft_angle_rad_from_raw(&self, raw: u16) -> f32 {
        Gim6010::<DEV_ADDR>::raw_angle_to_rad(raw)
    }

    /// Convert a raw encoder value [0..65535] into motor shaft angle in degrees.
    #[inline]
    pub fn shaft_angle_deg_from_raw(&self, raw: u16) -> f32 {
        Gim6010::<DEV_ADDR>::raw_angle_to_deg(raw)
    }

    /// Convert a desired motor shaft angle (radians) into a raw encoder value.
    #[inline]
    pub fn raw_for_shaft_angle_rad(&self, angle_rad: f32) -> u16 {
        Gim6010::<DEV_ADDR>::angle_rad_to_raw(angle_rad)
    }

    /// Convert a desired motor shaft angle (degrees) into a raw encoder value.
    #[inline]
    pub fn raw_for_shaft_angle_deg(&self, angle_deg: f32) -> u16 {
        Gim6010::<DEV_ADDR>::angle_deg_to_raw(angle_deg)
    }

    /// Convert a raw encoder value into tile tilt angle in radians.
    #[inline]
    pub fn tile_angle_rad_from_raw(&self, raw: u16) -> f32 {
        let shaft = self.shaft_angle_rad_from_raw(raw);
        shaft / self.gear_ratio
    }

    /// Convert a raw encoder value into tile tilt angle in degrees.
    #[inline]
    pub fn tile_angle_deg_from_raw(&self, raw: u16) -> f32 {
        let shaft_deg = self.shaft_angle_deg_from_raw(raw);
        shaft_deg / self.gear_ratio
    }

    /// Convert a desired tile tilt angle in radians into a raw encoder value.
    #[inline]
    pub fn raw_for_tile_angle_rad(&self, tile_angle_rad: f32) -> u16 {
        let shaft = tile_angle_rad * self.gear_ratio;
        self.raw_for_shaft_angle_rad(shaft)
    }

    /// Convert a desired tile tilt angle in degrees into a raw encoder value.
    #[inline]
    pub fn raw_for_tile_angle_deg(&self, tile_angle_deg: f32) -> u16 {
        let shaft_deg = tile_angle_deg * self.gear_ratio;
        self.raw_for_shaft_angle_deg(shaft_deg)
    }
}
