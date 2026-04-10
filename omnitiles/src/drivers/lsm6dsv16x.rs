// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! ST LSM6DSV16X 6-axis IMU (accelerometer + gyroscope) SPI driver.
//!
//! Wired on SPI4 CS2. Uses 4-wire SPI mode 0 (CPOL=0, CPHA=0), matching the bus
//! config used by the DWM tag on CS1. The `read_sample()` method returns a single
//! synchronized accelerometer + gyroscope reading converted to SI units (m/s^2 and
//! rad/s) so the tilt control loop on the STM32 can consume it directly.

use crate::hw::{spi::CsControl, SpiBus};
use core::f32::consts::PI;
use stm32f7xx_hal::spi;

/// Expected WHO_AM_I register value for the LSM6DSV16X.
const WHO_AM_I_VALUE: u8 = 0x70;

/// Standard gravity (m/s^2) for converting raw accel LSB to SI units.
const G: f32 = 9.80665;

mod reg {
    pub const WHO_AM_I: u8 = 0x0F;
    pub const CTRL1: u8 = 0x10;
    pub const CTRL2: u8 = 0x11;
    pub const CTRL3: u8 = 0x12;
    pub const CTRL6: u8 = 0x15;
    pub const CTRL8: u8 = 0x17;
    pub const OUTX_L_G: u8 = 0x22;
}

/// SPI read bit (OR'd into the register address byte).
const SPI_READ: u8 = 0x80;

#[derive(Debug)]
pub enum Error {
    Spi(spi::Error),
    InvalidDevice,
}

impl From<spi::Error> for Error {
    fn from(e: spi::Error) -> Self {
        Error::Spi(e)
    }
}

/// A single synchronized IMU sample in SI units.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuSample {
    /// Linear acceleration in m/s^2 (sensor frame).
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    /// Angular rate in rad/s (sensor frame).
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
}

/// LSM6DSV16X driver. Does not own the SPI bus or CS line — they are passed in
/// per-call so that SPI4 can be shared with other devices (DWM tag on CS1).
pub struct Lsm6dsv16x {
    accel_scale: f32,
    gyro_scale: f32,
}

impl Lsm6dsv16x {
    /// Probe WHO_AM_I, soft-reset, and configure accel + gyro for ±2 g / ±2000 dps
    /// at 120 Hz ODR in high-performance mode.
    pub fn new<I, PINS, CS>(spi: &mut SpiBus<I, PINS>, cs: &mut CS) -> Result<Self, Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
        CS: CsControl,
    {
        let mut dev = Self {
            accel_scale: (2.0 * G) / 32768.0,            // ±2 g full scale
            gyro_scale: (2000.0 * PI / 180.0) / 32768.0, // ±2000 dps full scale
        };

        if dev.read_reg(spi, cs, reg::WHO_AM_I)? != WHO_AM_I_VALUE {
            return Err(Error::InvalidDevice);
        }

        // CTRL3: BOOT=0, BDU=1 (bit6), IF_INC=1 (bit2), SW_RESET=1 (bit0).
        dev.write_reg(spi, cs, reg::CTRL3, 0x45)?;
        // Wait for SW_RESET to clear (datasheet: ~50 µs). Poll a few times.
        for _ in 0..1000 {
            let v = dev.read_reg(spi, cs, reg::CTRL3)?;
            if (v & 0x01) == 0 {
                break;
            }
        }
        // Ensure BDU + IF_INC are set after reset.
        dev.write_reg(spi, cs, reg::CTRL3, 0x44)?;

        // CTRL8: FS_XL[1:0] = 00 → ±2 g.
        dev.write_reg(spi, cs, reg::CTRL8, 0x00)?;
        // CTRL6: FS_G[3:0] = 0100 → ±2000 dps.
        dev.write_reg(spi, cs, reg::CTRL6, 0x04)?;

        // CTRL1: OP_MODE_XL[6:4] = 000 (high-perf), ODR_XL[3:0] = 0110 (120 Hz).
        dev.write_reg(spi, cs, reg::CTRL1, 0x06)?;
        // CTRL2: OP_MODE_G[6:4]  = 000 (high-perf), ODR_G[3:0]  = 0110 (120 Hz).
        dev.write_reg(spi, cs, reg::CTRL2, 0x06)?;

        Ok(dev)
    }

    /// Burst-read gyro XYZ and accel XYZ (12 bytes starting at OUTX_L_G) and convert
    /// to SI units. Gyro comes first in the register map, then accel.
    pub fn read_sample<I, PINS, CS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        cs: &mut CS,
    ) -> Result<ImuSample, Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
        CS: CsControl,
    {
        let mut buf = [0u8; 13];
        buf[0] = reg::OUTX_L_G | SPI_READ;

        cs.select();
        let res = spi.transfer_in_place(&mut buf);
        cs.deselect();
        res?;

        let gx_raw = i16::from_le_bytes([buf[1], buf[2]]);
        let gy_raw = i16::from_le_bytes([buf[3], buf[4]]);
        let gz_raw = i16::from_le_bytes([buf[5], buf[6]]);
        let ax_raw = i16::from_le_bytes([buf[7], buf[8]]);
        let ay_raw = i16::from_le_bytes([buf[9], buf[10]]);
        let az_raw = i16::from_le_bytes([buf[11], buf[12]]);

        Ok(ImuSample {
            ax: ax_raw as f32 * self.accel_scale,
            ay: ay_raw as f32 * self.accel_scale,
            az: az_raw as f32 * self.accel_scale,
            gx: gx_raw as f32 * self.gyro_scale,
            gy: gy_raw as f32 * self.gyro_scale,
            gz: gz_raw as f32 * self.gyro_scale,
        })
    }

    fn read_reg<I, PINS, CS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        cs: &mut CS,
        addr: u8,
    ) -> Result<u8, Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
        CS: CsControl,
    {
        let mut buf = [addr | SPI_READ, 0];
        cs.select();
        let res = spi.transfer_in_place(&mut buf);
        cs.deselect();
        res?;
        Ok(buf[1])
    }

    fn write_reg<I, PINS, CS>(
        &mut self,
        spi: &mut SpiBus<I, PINS>,
        cs: &mut CS,
        addr: u8,
        value: u8,
    ) -> Result<(), Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
        CS: CsControl,
    {
        let mut buf = [addr & 0x7F, value];
        cs.select();
        let res = spi.transfer_in_place(&mut buf);
        cs.deselect();
        res?;
        Ok(())
    }
}
