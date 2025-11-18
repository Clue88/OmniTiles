//! # Chip-level / Board-level Drivers
//!
//! This module contains device-specific drivers that sit above the raw `hw/` layer and below the
//! application logic.
//!
//! ## Existing drivers
//!
//! - [`drv8873`] – TI DRV8873-Q1 4-wire SPI motor driver
//! - [`spi_motor`] – High-level motor wrapper combining a `Drv8873` and a
//!   32-bit TIM2 quadrature [`crate::hw::Encoder`].

pub mod drv8873;
pub mod spi_motor;

pub use drv8873::Drv8873;
pub use spi_motor::SpiMotor;
