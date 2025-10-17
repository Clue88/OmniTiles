//! PowerSTEP01 SPI driver helpers for the X-NUCLEO-IHM03A1 expansion board.
//!
//! This module provides basic communication routines with the PowerSTEP01
//! stepper driver using the STM32F7 HAL.
//!
//! Currently it supports:
//! - Reading the 16-bit `STATUS` register via the `GetStatus (0xD0)` command.

use hal::prelude::*;
use stm32f7xx_hal as hal;

/// Simple trait for any GPIO pin used as PowerSTEP01 chip select (CS).
pub trait CsPin {
    fn low(&mut self);
    fn high(&mut self);
}

/// Implement [`CsPin`] for PD14 (Arduino D10) — the default CS line when the
/// X-NUCLEO-IHM03A1 is stacked on a NUCLEO-F767ZI.
impl CsPin for hal::gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>> {
    #[inline]
    fn low(&mut self) {
        self.set_low();
    }
    #[inline]
    fn high(&mut self) {
        self.set_high();
    }
}

/// Perform a blocking full-duplex SPI transfer in-place.
///
/// Sends each byte of `data` and overwrites it with the simultaneously received byte.
fn spi_xfer_in_place<I, P>(spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>, data: &mut [u8])
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
{
    for b in data.iter_mut() {
        let _ = nb::block!(spi.send(*b));
        match nb::block!(spi.read()) {
            Ok(rx) => *b = rx,
            Err(_) => { /* leave byte unchanged on error */ }
        }
    }
}

/// Issue the **GetStatus (0xD0)** command to the PowerSTEP01 and return the 16-bit `STATUS`
/// register value.
///
/// # Returns
/// A 16-bit status value (MSB first).
pub fn get_status<I, P, CS>(
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
) -> u16
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: CsPin,
{
    // 0xD0 is the GetStatus command; the register value is placed in the following 2B
    let mut buf = [0xD0, 0x00, 0x00];

    cs.low();
    spi_xfer_in_place(spi, &mut buf);
    cs.high();

    ((buf[1] as u16) << 8) | (buf[2] as u16)
}
