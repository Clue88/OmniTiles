//! DRV8873-Q1 SPI motor driver.
//!
//! This module handles SPI framing and register access for DRV8873-Q1. Higher-level motor control
//! can be layered on top of these primitives.

use crate::hw::{ChipSelect, SpiBus};
use stm32f7xx_hal::spi;

// Register addresses
pub mod reg {
    pub const FAULT: u8 = 0x00;
    pub const DIAG: u8 = 0x01;
    pub const IC1: u8 = 0x02;
    pub const IC2: u8 = 0x03;
    pub const IC3: u8 = 0x04;
    pub const IC4: u8 = 0x05;
}

/// Status byte returned in the upper 8 bits of SDO.
#[derive(Copy, Clone, Debug)]
pub struct Status {
    raw: u8,
}

impl Status {
    #[inline]
    pub fn raw(&self) -> u8 {
        self.raw
    }

    /// Overtemperature warning.
    #[inline]
    pub fn otw(&self) -> bool {
        (self.raw & (1 << 5)) != 0
    }

    /// UVLO fault condition.
    #[inline]
    pub fn uvlo(&self) -> bool {
        (self.raw & (1 << 4)) != 0
    }

    /// Charge-pump undervoltage fault condition.
    #[inline]
    pub fn cpuv(&self) -> bool {
        (self.raw & (1 << 3)) != 0
    }

    /// Overcurrent condition.
    #[inline]
    pub fn ocp(&self) -> bool {
        (self.raw & (1 << 2)) != 0
    }

    /// Overcurrent shutdown.
    #[inline]
    pub fn tsd(&self) -> bool {
        (self.raw & (1 << 1)) != 0
    }

    /// Open-load detection.
    #[inline]
    pub fn old(&self) -> bool {
        (self.raw & (1 << 0)) != 0
    }
}

/// FAULT status register.
#[derive(Copy, Clone, Debug)]
pub struct Fault {
    raw: u8,
}

impl Fault {
    #[inline]
    pub fn raw(&self) -> u8 {
        self.raw
    }

    /// Global FAULT status register. Complements the nFAULT pin.
    #[inline]
    pub fn fault(&self) -> bool {
        (self.raw & (1 << 6)) != 0
    }

    /// Overtemperature warning.
    #[inline]
    pub fn otw(&self) -> bool {
        (self.raw & (1 << 5)) != 0
    }

    /// UVLO fault condition.
    #[inline]
    pub fn uvlo(&self) -> bool {
        (self.raw & (1 << 4)) != 0
    }

    /// Charge-pump undervoltage fault condition.
    #[inline]
    pub fn cpuv(&self) -> bool {
        (self.raw & (1 << 3)) != 0
    }

    /// Overcurrent condition.
    #[inline]
    pub fn ocp(&self) -> bool {
        (self.raw & (1 << 2)) != 0
    }

    /// Overcurrent shutdown.
    #[inline]
    pub fn tsd(&self) -> bool {
        (self.raw & (1 << 1)) != 0
    }

    /// Open-load detection.
    #[inline]
    pub fn old(&self) -> bool {
        (self.raw & (1 << 0)) != 0
    }
}

/// DIAG status register.
#[derive(Copy, Clone, Debug)]
pub struct Diag {
    raw: u8,
}

impl Diag {
    #[inline]
    pub fn raw(&self) -> u8 {
        self.raw
    }

    /// Open-load detection on half bridge 1.
    #[inline]
    pub fn ol1(&self) -> bool {
        (self.raw & (1 << 7)) != 0
    }

    /// Open-load detection on half bridge 2.
    #[inline]
    pub fn ol2(&self) -> bool {
        (self.raw & (1 << 6)) != 0
    }

    /// Current regulation status of half bridge 1.
    ///
    /// 1 indicates output 1 is in current regulation, 0 indicates it is not.
    #[inline]
    pub fn itrip1(&self) -> bool {
        (self.raw & (1 << 5)) != 0
    }

    /// Current regulation status of half bridge 2.
    ///
    /// 1 indicates output 2 is in current regulation, 0 indicates it is not.
    #[inline]
    pub fn itrip2(&self) -> bool {
        (self.raw & (1 << 4)) != 0
    }

    /// Overcurrent fault on the high-side FET of half bridge 1.
    #[inline]
    pub fn ocp_h1(&self) -> bool {
        (self.raw & (1 << 3)) != 0
    }

    /// Overcurrent fault on the low-side FET of half bridge 1.
    #[inline]
    pub fn ocp_l1(&self) -> bool {
        (self.raw & (1 << 2)) != 0
    }

    /// Overcurrent fault on the high-side FET of half bridge 2.
    #[inline]
    pub fn ocp_h2(&self) -> bool {
        (self.raw & (1 << 1)) != 0
    }

    /// Overcurrent fault on the low-side FET of half bridge 2.
    #[inline]
    pub fn ocp_l2(&self) -> bool {
        (self.raw & 1) != 0
    }
}

/// Response of a single SPI transaction:
/// - status byte (fault/warning flags)
/// - data byte (register contents)
#[derive(Copy, Clone, Debug)]
pub struct Response {
    pub status: Status,
    pub data: u8,
}

/// DRV8873 driver bound to a specific chip-select pin.
///
/// The SPI bus is passed in as &mut to each method so that multiple DRV8873 instances can share the
/// same bus.
pub struct Drv8873<const P: char, const N: u8> {
    cs: ChipSelect<P, N>,
}

impl<const P: char, const N: u8> Drv8873<P, N> {
    /// Construct a driver from an active-low chip-select pin.
    pub fn new(cs: ChipSelect<P, N>) -> Self {
        Self { cs }
    }

    /// Release the chip-select pin.
    pub fn free(self) -> ChipSelect<P, N> {
        self.cs
    }

    /// Build a 16-bit SPI word for this device.
    /// - `is_read`: true for read, false for write
    /// - `addr`: 5-bit register address
    /// - `data`: 8-bit data payload (ignored for reads by the device)
    #[inline]
    fn build_word(is_read: bool, addr: u8, data: u8) -> u16 {
        let mut word: u16 = 0;

        // B15 = 0
        // B14 = W (1 = read, 0 = write)
        if is_read {
            word |= 1 << 14;
        }

        // B13..B9 = A4..A0 (5-bit addr)
        word |= ((addr as u16) & 0x1F) << 9;

        // B8 = X
        // B7..B0 = data
        word |= data as u16;

        word
    }

    /// Send a 16-bit word and receive the status + data bytes.
    /// - `spi` must be an enabled 8-bit SPI bus configured in the correct mode
    ///   for the DRV8873 (CPOL=0, CPHA=1: data captured on falling edge, driven on rising edge).
    pub fn transfer_word<I, PINS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        word: u16,
    ) -> Result<Response, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        let mut buf = [(word >> 8) as u8, word as u8];

        self.cs.select();
        spi.transfer_in_place(&mut buf)?;
        self.cs.deselect();

        let status = Status { raw: buf[0] };
        let data = buf[1];

        Ok(Response { status, data })
    }

    /// Write a register and return the response (status + current register contents).
    pub fn write_reg<I, PINS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        addr: u8,
        value: u8,
    ) -> Result<Response, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        let word = Self::build_word(false, addr, value);
        self.transfer_word(spi, word)
    }

    /// Read a register and return the response (status + register value).
    pub fn read_reg<I, PINS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        addr: u8,
    ) -> Result<Response, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        let word = Self::build_word(true, addr, 0x00);
        self.transfer_word(spi, word)
    }

    /// Read the FAULT register and parse into a `Fault` struct.
    ///
    /// To get the status result as well, use `read_reg`.
    pub fn read_fault<I, PINS>(&mut self, spi: &mut SpiBus<I, PINS>) -> Result<Fault, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        Ok(Fault {
            raw: self.read_reg(spi, reg::FAULT)?.data,
        })
    }

    /// Read the DIAG register and parse into a `Diag` struct.
    ///
    /// To get the status result as well, use `read_reg`.
    pub fn read_diag<I, PINS>(&mut self, spi: &mut SpiBus<I, PINS>) -> Result<Diag, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        Ok(Diag {
            raw: self.read_reg(spi, reg::DIAG)?.data,
        })
    }
}
