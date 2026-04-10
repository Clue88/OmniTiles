// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # Control Algorithms
//!
//! This module provides reusable building blocks for closed-loop motor control.
//!
//! ## Modules
//!
//! - [`pid`] - General-purpose PID controller implementation.
//! - [`linear_controller`] - Closed-loop position controller for Actuonix linear actuators.

pub mod base_controller;
pub mod linear_controller;
pub mod mecanum;
pub mod pid;

pub use base_controller::BaseController;
pub use linear_controller::{LinearController, LinearMode};
pub use pid::Pid;
