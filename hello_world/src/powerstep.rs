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

/// Clock one byte (send `tx`) and return the received byte.
#[inline]
fn spi_send_recv_byte<I, P>(
    spi: &mut stm32f7xx_hal::spi::Spi<I, P, stm32f7xx_hal::spi::Enabled<u8>>,
    tx: u8,
) -> u8
where
    I: stm32f7xx_hal::spi::Instance,
    P: stm32f7xx_hal::spi::Pins<I>,
{
    let _ = nb::block!(spi.send(tx));
    nb::block!(spi.read()).unwrap_or(0)
}

/// Issue the GetStatus (0xD0) command to the PowerSTEP01 and return the 16-bit `STATUS`
/// register value.
///
/// The status is returned in the two bytes after the command (MSB first).
pub fn get_status<I, P, CS>(
    spi: &mut stm32f7xx_hal::spi::Spi<I, P, stm32f7xx_hal::spi::Enabled<u8>>,
    cs: &mut CS,
) -> u16
where
    I: stm32f7xx_hal::spi::Instance,
    P: stm32f7xx_hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    cs.low();

    // Send CMD; discard pipelined byte read during the command
    let _rx0 = spi_send_recv_byte(spi, 0xD0);

    // Clock out two NOPs to read status MSB and LSB
    let msb = spi_send_recv_byte(spi, 0x00);
    let lsb = spi_send_recv_byte(spi, 0x00);

    cs.high();

    ((msb as u16) << 8) | (lsb as u16)
}

/// Read a parameter from PowerSTEP01.
/// * `reg`: 5-bit register code (masked with 0x1F)
/// * `len`: number of data bytes to read (1..=4), **MSB first** on the wire.
/// Returns the value in the low bits of `u32`.
pub fn get_param<I, P, CS>(
    spi: &mut stm32f7xx_hal::spi::Spi<I, P, stm32f7xx_hal::spi::Enabled<u8>>,
    cs: &mut CS,
    reg: u8,
    len: u8,
) -> u32
where
    I: stm32f7xx_hal::spi::Instance,
    P: stm32f7xx_hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let n = len.clamp(1, 4);
    let opcode = 0x20 | (reg & 0x1F); // GET_PARAM = 0b001xxxxx

    cs.low();

    // Send opcode; discard pipelined rx
    let _ = spi_send_recv_byte(spi, opcode);

    // Read `n` data bytes (MSB first), clocking NOPs
    let mut value: u32 = 0;
    for _ in 0..n {
        let b = spi_send_recv_byte(spi, 0x00);
        value = (value << 8) | (b as u32);
    }

    cs.high();
    value
}

/// Write a parameter to PowerSTEP01.
/// * `reg`: 5-bit register code (masked with 0x1F)
/// * `value`: value to write (lower `len*8` bits used)
/// * `len`: number of data bytes (1..=4), sent **MSB first**
pub fn set_param<I, P, CS>(
    spi: &mut stm32f7xx_hal::spi::Spi<I, P, stm32f7xx_hal::spi::Enabled<u8>>,
    cs: &mut CS,
    reg: u8,
    value: u32,
    len: u8,
) where
    I: stm32f7xx_hal::spi::Instance,
    P: stm32f7xx_hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let n = len.clamp(1, 4) as u32;
    let opcode = reg & 0x1F; // SET_PARAM = 0b000xxxxx

    cs.low();

    // Send opcode; discard pipelined rx
    let _ = spi_send_recv_byte(spi, opcode);

    // Send value MSB-first
    let total_bits = n * 8;
    let mut shift = total_bits.saturating_sub(8);
    for _ in 0..n {
        let b = ((value >> shift) & 0xFF) as u8;
        let _ = spi_send_recv_byte(spi, b);
        shift = shift.saturating_sub(8);
    }

    cs.high();
}
