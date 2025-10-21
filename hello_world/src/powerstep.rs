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
#[inline]
fn spi_send_recv_byte<I, P>(spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>, tx: u8) -> u8
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
{
    let _ = nb::block!(spi.send(tx));
    nb::block!(spi.read()).unwrap_or(0)
}

/// Send the ResetDevice command to PowerSTEP01. The ResetDevice command resets the device to power-up
/// conditions.
pub fn reset_device<I, P, CS>(spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>, cs: &mut CS)
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let opcode = 0xC0;

    cs.low();
    let _ = spi_send_recv_byte(spi, opcode);
    cs.high();
    cortex_m::asm::nop();
}

/// Send the GetStatus command to PowerSTEP01. The GetStatus command resets the STATUS register
/// warning flags. The command forces the system to exit from any error state. The GetStatus command
/// does not reset the HiZ flag.
///
/// Returns the STATUS register value.
pub fn get_status<I, P, CS>(
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
) -> u16
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let opcode = 0xD0;

    cs.low();
    let _ = spi_send_recv_byte(spi, opcode);
    cs.high();

    cortex_m::asm::nop();
    cs.low();
    let b_hi = spi_send_recv_byte(spi, 0x00);
    cs.high();

    cortex_m::asm::nop();
    cs.low();
    let b_lo = spi_send_recv_byte(spi, 0x00);
    cs.high();

    ((b_hi as u16) << 8) | (b_lo as u16)
}

/// Read a parameter from PowerSTEP01.
/// * `reg`: 5-bit register code (masked with 0x1F)
/// * `len`: number of data bytes to read (1..=4)
/// Returns the value in the low bits of `u32`.
pub fn get_param<I, P, CS>(
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
    reg: u8,
    len: u8,
) -> u32
where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let opcode = 0x20 | (reg & 0x1F); // GET_PARAM opcode (0b001xxxxx)
    let n = len.clamp(1, 4) as u32;

    cortex_m::asm::nop();
    cs.low();
    let _ = spi_send_recv_byte(spi, opcode);
    cs.high();

    let mut val: u32 = 0;
    for _ in 0..n {
        cortex_m::asm::nop();
        cs.low();
        let b = spi_send_recv_byte(spi, 0x00);
        val = (val << 8) | (b as u32);
        cs.high();
    }
    val
}

/// Write a parameter to PowerSTEP01.
/// * `reg`: 5-bit register code (masked with 0x1F)
/// * `value`: value to write (lower `len*8` bits used)
/// * `len`: number of data bytes (1..=4)
pub fn set_param<I, P, CS>(
    spi: &mut hal::spi::Spi<I, P, hal::spi::Enabled<u8>>,
    cs: &mut CS,
    reg: u8,
    val: u32,
    len: u8,
) where
    I: hal::spi::Instance,
    P: hal::spi::Pins<I>,
    CS: crate::powerstep::CsPin,
{
    let opcode = reg & 0x1F; // SET_PARAM opcode (0b000xxxxx)
    let n = len.clamp(1, 4) as u32;

    cs.low();
    let _ = spi_send_recv_byte(spi, opcode);
    cs.high();

    let total_bits = n * 8;
    let mut shift = total_bits.saturating_sub(8);
    for _ in 0..n {
        cortex_m::asm::nop();
        cs.low();
        let b = ((val >> shift) & 0xFF) as u8;
        let _ = spi_send_recv_byte(spi, b);
        shift = shift.saturating_sub(8);
        cs.high();
    }
}
