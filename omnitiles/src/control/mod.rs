//! # Control Algorithms
//!
//! This module provides reusable building blocks for closed-loop motor control.
//!
//! ## Modules
//!
//! - [`pid`] - General-purpose PID controller implementation.
//! - [`lift_controller`] - Closed-loop height controller for the linear lift actuator.

pub mod lift_controller;
pub mod pid;

pub use lift_controller::{LiftController, LiftMode};
pub use pid::Pid;
