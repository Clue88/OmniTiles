//! # Actuator Abstractions
//!
//! This module contains motor-level wrappers that sit above device-level drivers in `drivers`.
//!
//! ## Modules
//!
//! - [`lift_motor`] - Lift actuator built on `SpiMotor` (DRV8873 + encoder)

pub mod lift_motor;
