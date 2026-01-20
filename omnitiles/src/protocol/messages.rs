// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Command message protocol used to communicate with OmniTiles.

/// Sync byte for the protocol.
pub const START_BYTE: u8 = 0xA5;

// Message IDs
pub const MSG_FIT0185_FORWARD: u8 = 0x30;
pub const MSG_FIT0185_REVERSE: u8 = 0x31;
pub const MSG_FIT0185_BRAKE: u8 = 0x32;

/// Direct motor commands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Fit0185Forward,
    Fit0185Reverse,
    Fit0185Brake,
}
