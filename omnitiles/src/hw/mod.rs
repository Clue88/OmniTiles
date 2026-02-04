// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # Hardware Abstraction Layer (HAL‐level board support)
//!
//! This module contains low-level, MCU-specific abstractions for the STM32F777 that are safe, thin
//! wrappers over PAC (peripheral access crate) registers or `stm32f7xx-hal` drivers.
//!
//! ## Modules
//!
//! - [`pins_v1`] - OmniTiles STM32F777 pin assignments for PCB v1
//! - [`led`] – Active-high / active-low LED wrapper
//! - [`usart`] – Blocking TX helpers with `core::fmt::Write` impl
//! - [`spi`] – Blocking byte-level SPI and reusable CS abstraction
//! - [`can`] – Safe wrapper around `bxcan` with blocking send/receive
//! - [`encoder`] – TIM2/TIM3 quadrature encoder mode
//! - [`adc`] – ADC1/ADC2/ADC3 single-channel blocking reads

pub mod adc;
pub mod can;
pub mod encoder;
pub mod led;
pub mod pins_devboard;
pub mod pins_v1;
pub mod spi;
pub mod usart;

pub use adc::Adc;
pub use can::CanBus;
pub use encoder::Encoder;
pub use led::Led;
pub use pins_v1::BoardPins;
pub use spi::ChipSelect;
pub use spi::SpiBus;
pub use usart::Usart;
