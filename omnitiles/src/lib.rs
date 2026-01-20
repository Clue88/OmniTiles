// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # OmniTiles Firmware
//!
//! This crate contains all firmware components for the OmniTiles robotics platform, written in
//! Rust, targeting an STM32F777 MCU.
//!
//! ## Crate Structure
//!
//! | Module | Purpose |
//! | ------ | -------- |
//! | [`hw`] | MCU-level wrappers around USART, SPI, CAN, timers, etc. |
//! | [`drivers`] | Device-level drivers (e.g., DRV8873, GDZ468) |
//! | [`motors`]  | Actuator-level structures (lift, tilt) built on top of drivers |
//! | [`control`]   | Control algorithms (PID, high-level control) |
//!
//! ## Getting Started
//!
//! Build docs:
//!
//! ```bash
//! cargo doc --no-deps --open
//! ```
//!
//! Flash the board:
//!
//! ```bash
//! cargo run --release
//! ```
//!
//! ## Hardware Notes
//!
//! * **MCU:** STM32F777VI
//! * **Debug UART:** USART1 @ 115200 baud
//! * **Lift Motor:** FIT0185 with DRV8873 driver over SPI4 and TIM2 32-bit quadrature encoder
//! * **Tilt Motor:** GIM6010-48 over CAN2 with built-in GDZ468 encoder
//!
//! ## License
//!
//! Licensed under the **MIT License**.
//! See the `LICENSE` file in the repository root for full terms.
//!
//! © 2025–2026 Christopher Liu

#![no_std]

pub mod control;
pub mod drivers;
pub mod hw;
pub mod motors;
pub mod protocol;
