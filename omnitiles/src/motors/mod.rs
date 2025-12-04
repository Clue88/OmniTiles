// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # Actuator Abstractions
//!
//! This module contains motor-level wrappers that sit above device-level drivers in `drivers`.
//!
//! ## Modules
//!
//! - [`lift_motor`] - Lift actuator built on `SpiMotor` (DRV8873 + encoder).
//! - [`tilt_motor`] - Tilt actuator built on `CanMotor`.

pub mod lift_motor;
pub mod tilt_motor;

pub use lift_motor::LiftMotor;
pub use tilt_motor::TiltMotor;
