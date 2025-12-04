// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! USART abstraction layer.
//!
//! Provides several printing helpers for hex, decimal, and ASCII strings to print to an attached
//! debug terminal.
//!
//! Note: When using `writeln!`, be sure to include `\r` (CR) in the format string to ensure correct
//! line endings on the terminal.
//!
//! To access the terminal on the host machine, connect to the debug USB port and use
//! ```
//! $ screen /dev/tty.usbmodem* <baud_rate>
//! ```
//!
//! To close the debug terminal, press `Ctrl+A` then `Ctrl+\` then `y`.

use core::fmt;
use nb::block;

use stm32f7xx_hal::{
    prelude::*,
    serial::{Instance, Pins, Serial, Tx},
};

pub struct Usart<U: Instance> {
    tx: Tx<U>,
}

impl<U: Instance> Usart<U> {
    pub fn new<PINS: Pins<U>>(serial: Serial<U, PINS>) -> Self {
        let (tx, _rx) = serial.split();
        Self { tx }
    }

    #[inline]
    pub fn write_byte(&mut self, b: u8) {
        let _ = block!(self.tx.write(b));
    }

    pub fn write_str(&mut self, s: &str) {
        for &b in s.as_bytes() {
            self.write_byte(b);
        }
    }

    /// Write string and CRLF terminator.
    #[inline]
    pub fn println(&mut self, s: &str) {
        self.write_str(s);
        self.write_str("\r\n");
    }

    /// Block until the hardware TX FIFO/drain is flushed.
    #[inline]
    pub fn flush(&mut self) {
        let _ = block!(self.tx.flush());
    }

    pub fn print_hex_u8(&mut self, n: u8) {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        self.write_str("0x");
        self.write_byte(HEX[((n >> 4) & 0xF) as usize]);
        self.write_byte(HEX[(n & 0xF) as usize]);
    }

    pub fn print_hex_u16(&mut self, n: u16) {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        self.write_str("0x");
        for shift in (0..=12).rev().step_by(4) {
            self.write_byte(HEX[((n >> shift) & 0xF) as usize]);
        }
    }

    pub fn print_hex_u32(&mut self, n: u32) {
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        self.write_str("0x");
        for (i, shift) in (0..=28).rev().step_by(4).enumerate() {
            if i == 4 {
                self.write_byte(b'_');
            }
            self.write_byte(HEX[((n >> shift) & 0xF) as usize]);
        }
    }

    pub fn print_u32(&mut self, mut n: u32) {
        let mut buf = [0u8; 10];
        let mut i = buf.len();
        if n == 0 {
            self.write_byte(b'0');
            return;
        }
        while n > 0 {
            i -= 1;
            buf[i] = b'0' + (n % 10) as u8;
            n /= 10;
        }
        for &b in &buf[i..] {
            self.write_byte(b);
        }
    }
}

// Implement `core::fmt::Write` so we can use `write!` / `writeln!` on `Usart`.
impl<U: Instance> fmt::Write for Usart<U> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        Usart::write_str(self, s);
        Ok(())
    }
}
