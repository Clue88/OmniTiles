// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # Device-Specific Drivers
//!
//! This module contains device-specific drivers that sit above the raw `hw/` layer and below the
//! application logic.
//!
//! ## Existing drivers
//!
//! - [`drv8873`] – TI DRV8873-Q1 4-wire SPI motor driver
//! - [`actuonix_linear`] – Actuonix linear actuator with DRV8873 driver and potentiometer feedback
//! - [`vl53l0x`] – VL53L0X Time-of-Flight sensor
//!
//! ## Legacy drivers
//!
//! - [`fit0185`] – DFRobot FIT0185 motor with DRV8873 driver and TIM2 encoder
//! - [`gim6010`] – SteadyWin GIM6010-48 motor with built-in GDZ468 encoder

pub mod drv8873;

pub mod actuonix_linear;
pub mod fit0185;
pub mod gim6010;
pub mod tb6612;
pub mod vl53l0x;

pub use actuonix_linear::ActuonixLinear;
pub use drv8873::Drv8873;
pub use fit0185::Fit0185;
pub use gim6010::Gim6010;
pub use tb6612::Tb6612;
pub use vl53l0x::Vl53l0x;
