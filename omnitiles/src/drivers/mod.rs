//! # Device-Specific Drivers
//!
//! This module contains device-specific drivers that sit above the raw `hw/` layer and below the
//! application logic.
//!
//! ## Existing drivers
//!
//! - [`drv8873`] – TI DRV8873-Q1 4-wire SPI motor driver
//! - [`fit0185`] – FIT0185 motor with DRV8873 driver and TIM2 encoder
//! - [`can_motor`] – High-level CAN motor (SteadyWin GIM6010 + GDZ468)

pub mod drv8873;

pub mod can_motor;
pub mod fit0185;

pub use can_motor::CanMotor;
pub use drv8873::Drv8873;
pub use fit0185::Fit0185;
