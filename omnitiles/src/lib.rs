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
pub mod protocol;
