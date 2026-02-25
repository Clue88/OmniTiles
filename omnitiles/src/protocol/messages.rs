// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Command message protocol used to communicate with OmniTiles.

/// Sync byte for the protocol.
pub const START_BYTE: u8 = 0xA5;

// Message IDs
pub const MSG_M1_EXTEND: u8 = 0x30;
pub const MSG_M1_RETRACT: u8 = 0x31;
pub const MSG_M1_BRAKE: u8 = 0x32;
pub const MSG_M1_SET_POSITION: u8 = 0x33;

pub const MSG_M2_EXTEND: u8 = 0x40;
pub const MSG_M2_RETRACT: u8 = 0x41;
pub const MSG_M2_BRAKE: u8 = 0x42;
pub const MSG_M2_SET_POSITION: u8 = 0x43;

pub const MSG_PING: u8 = 0x50;

pub const MSG_TELEMETRY: u8 = 0x60;

/// Direct motor commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Ping,
    M1Extend(u8),
    M1Retract(u8),
    M1Brake,
    M1SetPosition(u8),
    M2Extend(u8),
    M2Retract(u8),
    M2Brake,
    M2SetPosition(u8),
}
