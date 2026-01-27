// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Message parser for the OmniTiles command protocol.
//!
//! This module provides functionality to parse incoming command messages
//! and convert them into actionable commands for the OmniTiles system.

use crate::protocol::messages::*;

enum State {
    WaitStart,
    WaitId,
    WaitChecksum { id: u8 },
}

pub struct Parser {
    state: State,
    checksum: u8,
}

impl Parser {
    pub fn new() -> Self {
        Self {
            state: State::WaitStart,
            checksum: 0,
        }
    }

    /// Process a single incoming byte. Returns `Some(Command)` if a complete packet is received.
    pub fn push(&mut self, byte: u8) -> Option<Command> {
        match self.state {
            State::WaitStart => {
                if byte == START_BYTE {
                    self.state = State::WaitId;
                    self.checksum = 0;
                }
            }
            State::WaitId => {
                self.checksum = self.checksum.wrapping_add(byte);

                match byte {
                    // P16
                    MSG_P16_EXTEND | MSG_P16_RETRACT | MSG_P16_BRAKE |
                    // T16
                    MSG_T16_EXTEND | MSG_T16_RETRACT | MSG_T16_BRAKE => {
                        self.state = State::WaitChecksum { id: byte };
                    }
                    _ => {
                        // Unknown message ID, reset state
                        self.state = State::WaitStart;
                    }
                }
            }
            State::WaitChecksum { id } => {
                // Verify checksum
                let valid = byte == self.checksum;
                self.state = State::WaitStart; // Reset for next message

                if valid {
                    return match id {
                        MSG_P16_EXTEND => Some(Command::P16Extend),
                        MSG_P16_RETRACT => Some(Command::P16Retract),
                        MSG_P16_BRAKE => Some(Command::P16Brake),
                        MSG_T16_EXTEND => Some(Command::T16Extend),
                        MSG_T16_RETRACT => Some(Command::T16Retract),
                        MSG_T16_BRAKE => Some(Command::T16Brake),
                        _ => None,
                    };
                }
            }
        }
        None
    }
}
