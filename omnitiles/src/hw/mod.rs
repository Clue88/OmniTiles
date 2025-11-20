//! # Hardware Abstraction Layer (HAL‐level board support)
//!
//! This module contains low-level, MCU-specific abstractions for the STM32F777 that are safe, thin
//! wrappers over PAC (peripheral access crate) registers or `stm32f7xx-hal` drivers.
//!
//! ## Modules
//!
//! - [`pins`] - OmniTiles STM32F777 pin assignments
//! - [`led`] – Active-high / active-low LED wrapper
//! - [`usart`] – Blocking TX helpers with `core::fmt::Write` impl
//! - [`spi`] – Blocking byte-level SPI and reusable CS abstraction
//! - [`can`] – Safe wrapper around `bxcan` with blocking send/receive
//! - [`encoder`] – TIM2/TIM3 quadrature encoder mode

pub mod can;
pub mod encoder;
pub mod led;
pub mod pins;
pub mod pins_f767zi;
pub mod spi;
pub mod usart;

pub use can::CanBus;
pub use encoder::Encoder;
pub use led::Led;
pub use pins::BoardPins;
pub use spi::ChipSelect;
pub use spi::SpiBus;
pub use usart::Usart;
