// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Serial Peripheral Interface (SPI) abstraction layer.
//!
//! - `SpiBus` wraps a configured HAL SPI instance with 8-bit words.
//! - `ChipSelect` is an active-low GPIO output wrappr for manual CS control.

use stm32f7xx_hal::{
    gpio::{self, Output, PinState, PushPull},
    prelude::*,
    spi::{self, Enabled, Spi},
};

/// Wrapper around an enabled HAL SPI instance (8-bit words).
pub struct SpiBus<I, P> {
    spi: Spi<I, P, Enabled<u8>>,
}

impl<I, P> SpiBus<I, P>
where
    I: spi::Instance,
    P: spi::Pins<I>,
{
    pub fn new(spi: Spi<I, P, Enabled<u8>>) -> Self {
        Self { spi }
    }

    /// Perform a blocking, full-duplex transfer of one byte.
    pub fn transfer_byte(&mut self, byte: u8) -> Result<u8, spi::Error> {
        let mut tmp = [byte];
        self.spi.transfer(&mut tmp)?;
        Ok(tmp[0])
    }

    /// Send a byte, ignoring the response.
    #[inline]
    pub fn write_byte(&mut self, byte: u8) -> Result<(), spi::Error> {
        let _ = self.transfer_byte(byte)?;
        Ok(())
    }

    /// Read a byte, sending 0x00.
    #[inline]
    pub fn read_byte(&mut self) -> Result<u8, spi::Error> {
        self.transfer_byte(0x00)
    }

    /// Transfer a byte buffer in-place.
    pub fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), spi::Error> {
        for b in buf.iter_mut() {
            *b = self.transfer_byte(*b)?;
        }
        Ok(())
    }

    pub fn free(self) -> Spi<I, P, Enabled<u8>> {
        self.spi
    }
}

/// Manual chip-select line, active-low, generic over any GPIO pin.
pub struct ChipSelect<const P: char, const N: u8> {
    pin: gpio::Pin<P, N, Output<PushPull>>,
}

impl<const P: char, const N: u8> ChipSelect<P, N> {
    /// Create an active-low chip select and set to the inactive state (i.e., high).
    pub fn active_low<MODE>(pin: gpio::Pin<P, N, MODE>) -> Self {
        let mut pin = pin.into_push_pull_output();
        pin.set_state(PinState::High);
        Self { pin }
    }

    /// Assert the chip select.
    #[inline]
    pub fn select(&mut self) {
        self.pin.set_low();
    }

    /// Deassert the chip select.
    #[inline]
    pub fn deselect(&mut self) {
        self.pin.set_high();
    }

    pub fn free(self) -> gpio::Pin<P, N, Output<PushPull>> {
        self.pin
    }
}
