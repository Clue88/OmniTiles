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
//! - [`fit0185`] – DFRobot FIT0185 motor with DRV8873 driver and TIM2 encoder
//! - [`gim6010`] – SteadyWin GIM6010-48 motor with built-in GDZ468 encoder
//! - [`p16p`] – Actuonix P16-P linear actuator with DRV8873 driver and potentiometer feedback

pub mod drv8873;

pub mod fit0185;
pub mod gim6010;
pub mod p16p;

pub use drv8873::Drv8873;
pub use fit0185::Fit0185;
pub use gim6010::Gim6010;
pub use p16p::P16P;
