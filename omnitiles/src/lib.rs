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
//! * **Motor control:** DRV8873 via SPI4, two chip select lines
//! * **Position feedback:** TIM2 (32-bit) + TIM3 (16-bit) encoder mode
//! * **Debug UART:** USART1 @ 115200 baud
//! * **CAN:** CAN2 for real network, CAN1 owns filter banks
//!
//! ## License
//! MIT

#![no_std]

pub mod control;
pub mod drivers;
pub mod hw;
pub mod motors;
