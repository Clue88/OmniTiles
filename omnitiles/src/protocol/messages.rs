// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Command message protocol used to communicate with OmniTiles.

/// Sync byte for the protocol.
pub const START_BYTE: u8 = 0xA5;

// Message IDs
pub const MSG_P16_EXTEND: u8 = 0x30;
pub const MSG_P16_RETRACT: u8 = 0x31;
pub const MSG_P16_BRAKE: u8 = 0x32;

/// Direct motor commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    P16Extend,
    P16Retract,
    P16Brake,
}
