//! # Device-Specific Drivers
//!
//! This module contains device-specific drivers that sit above the raw `hw/` layer and below the
//! application logic.
//!
//! ## Existing drivers
//!
//! - [`drv8873`] – TI DRV8873-Q1 4-wire SPI motor driver
//! - [`spi_motor`] – High-level SPI motor (DRV8873 + TIM2 encoder)
//! - [`can_motor`] – High-level CAN motor (SteadyWin GIM6010 + GDZ468)

pub mod drv8873;

pub mod can_motor;
pub mod spi_motor;

pub use can_motor::CanMotor;
pub use spi_motor::SpiMotor;
