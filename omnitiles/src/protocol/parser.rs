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
    WaitPayload { id: u8 },
    WaitChecksum { id: u8, payload: Option<u8> },
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
                    // Messages with payload
                    MSG_P16_EXTEND | MSG_P16_RETRACT | MSG_T16_EXTEND | MSG_T16_RETRACT => {
                        self.state = State::WaitPayload { id: byte };
                    }
                    // Messages with no payload
                    MSG_P16_BRAKE | MSG_T16_BRAKE | MSG_PING => {
                        self.state = State::WaitChecksum {
                            id: byte,
                            payload: None,
                        };
                    }
                    _ => {
                        // Unknown message ID, reset state
                        self.state = State::WaitStart;
                    }
                }
            }
            State::WaitPayload { id } => {
                self.checksum = self.checksum.wrapping_add(byte);
                self.state = State::WaitChecksum {
                    id,
                    payload: Some(byte),
                };
            }
            State::WaitChecksum { id, payload } => {
                // Verify checksum
                let valid = byte == self.checksum;
                self.state = State::WaitStart; // Reset for next message

                if valid {
                    return match (id, payload) {
                        (MSG_P16_EXTEND, Some(p)) => Some(Command::P16Extend(p)),
                        (MSG_P16_RETRACT, Some(p)) => Some(Command::P16Retract(p)),
                        (MSG_P16_BRAKE, None) => Some(Command::P16Brake),
                        (MSG_T16_EXTEND, Some(p)) => Some(Command::T16Extend(p)),
                        (MSG_T16_RETRACT, Some(p)) => Some(Command::T16Retract(p)),
                        (MSG_T16_BRAKE, None) => Some(Command::T16Brake),
                        (MSG_PING, None) => Some(Command::Ping),
                        _ => None,
                    };
                }
            }
        }
        None
    }
}
