// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Message parser for the OmniTiles command protocol.
//!
//! This module provides functionality to parse incoming command messages
//! and convert them into actionable commands for the OmniTiles system.

use crate::protocol::messages::*;

/// Maximum payload size for any message.
const MAX_PAYLOAD: usize = 3;

enum State {
    WaitStart,
    WaitId,
    WaitPayload {
        id: u8,
        buf: [u8; MAX_PAYLOAD],
        received: u8,
        expected: u8,
    },
    WaitChecksum {
        id: u8,
        buf: [u8; MAX_PAYLOAD],
        len: u8,
    },
}

pub struct Parser {
    state: State,
    checksum: u8,
}

fn payload_len(id: u8) -> Option<u8> {
    match id {
        MSG_M1_EXTEND | MSG_M1_RETRACT | MSG_M1_SET_POSITION | MSG_M2_EXTEND | MSG_M2_RETRACT
        | MSG_M2_SET_POSITION => Some(1),
        MSG_M1_BRAKE | MSG_M2_BRAKE | MSG_PING | MSG_BASE_BRAKE => Some(0),
        MSG_BASE_VELOCITY => Some(3),
        _ => None,
    }
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

                match payload_len(byte) {
                    Some(0) => {
                        self.state = State::WaitChecksum {
                            id: byte,
                            buf: [0; MAX_PAYLOAD],
                            len: 0,
                        };
                    }
                    Some(n) => {
                        self.state = State::WaitPayload {
                            id: byte,
                            buf: [0; MAX_PAYLOAD],
                            received: 0,
                            expected: n,
                        };
                    }
                    None => {
                        self.state = State::WaitStart;
                    }
                }
            }
            State::WaitPayload {
                id,
                mut buf,
                received,
                expected,
            } => {
                self.checksum = self.checksum.wrapping_add(byte);
                buf[received as usize] = byte;
                let received = received + 1;

                if received >= expected {
                    self.state = State::WaitChecksum {
                        id,
                        buf,
                        len: received,
                    };
                } else {
                    self.state = State::WaitPayload {
                        id,
                        buf,
                        received,
                        expected,
                    };
                }
            }
            State::WaitChecksum { id, buf, len } => {
                let valid = byte == self.checksum;
                self.state = State::WaitStart;

                if valid {
                    return match id {
                        MSG_M1_EXTEND => Some(Command::M1Extend(buf[0])),
                        MSG_M1_RETRACT => Some(Command::M1Retract(buf[0])),
                        MSG_M1_BRAKE => Some(Command::M1Brake),
                        MSG_M1_SET_POSITION => Some(Command::M1SetPosition(buf[0])),
                        MSG_M2_EXTEND => Some(Command::M2Extend(buf[0])),
                        MSG_M2_RETRACT => Some(Command::M2Retract(buf[0])),
                        MSG_M2_BRAKE => Some(Command::M2Brake),
                        MSG_M2_SET_POSITION => Some(Command::M2SetPosition(buf[0])),
                        MSG_PING => Some(Command::Ping),
                        MSG_BASE_VELOCITY if len >= 3 => Some(Command::BaseVelocity {
                            vx: buf[0] as i8,
                            vy: buf[1] as i8,
                            omega: buf[2] as i8,
                        }),
                        MSG_BASE_BRAKE => Some(Command::BaseBrake),
                        _ => None,
                    };
                }
            }
        }
        None
    }
}
