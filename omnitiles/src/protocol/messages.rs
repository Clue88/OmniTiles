// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Command message protocol used to communicate with OmniTiles.

/// Sync byte for the protocol.
pub const START_BYTE: u8 = 0xA5;

// Message IDs
pub const MSG_P16_EXTEND: u8 = 0x30;
pub const MSG_P16_RETRACT: u8 = 0x31;
pub const MSG_P16_BRAKE: u8 = 0x32;

pub const MSG_T16_EXTEND: u8 = 0x40;
pub const MSG_T16_RETRACT: u8 = 0x41;
pub const MSG_T16_BRAKE: u8 = 0x42;

pub const MSG_PING: u8 = 0x50;

/// Direct motor commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Ping,
    P16Extend(u8),
    P16Retract(u8),
    P16Brake,
    T16Extend(u8),
    T16Retract(u8),
    T16Brake,
}
