// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! # Control Algorithms
//!
//! This module provides reusable building blocks for closed-loop motor control.
//!
//! ## Modules
//!
//! - [`pid`] - General-purpose PID controller implementation.
//! - [`lift_controller`] - Closed-loop height controller for the linear lift actuator.
//! - [`linear_controller`] - Closed-loop position controller for Actuonix linear actuators.

pub mod lift_controller;
pub mod linear_controller;
pub mod pid;

pub use lift_controller::{LiftController, LiftMode};
pub use linear_controller::{LinearController, LinearMode};
pub use pid::Pid;
