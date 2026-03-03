// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! I2C abstraction layer.
//!
//! Provides a thin blocking I2C bus wrapper around the stm32f7xx-hal `BlockingI2c` type.

use stm32f7xx_hal::{
    i2c::{self, BlockingI2c, PinScl, PinSda},
    pac::I2C1,
    prelude::*,
};

/// Wrapper around an enabled HAL BlockingI2c instance.
pub struct I2cBus<SCL, SDA> {
    i2c: BlockingI2c<I2C1, SCL, SDA>,
}

impl<SCL, SDA> I2cBus<SCL, SDA>
where
    SCL: PinScl<I2C1>,
    SDA: PinSda<I2C1>,
{
    pub fn new(i2c: BlockingI2c<I2C1, SCL, SDA>) -> Self {
        Self { i2c }
    }

    /// Perform a write-then-read transaction (register read pattern).
    pub fn write_read(&mut self, addr: u8, bytes: &[u8], buf: &mut [u8]) -> Result<(), i2c::Error> {
        self.i2c.write_read(addr, bytes, buf).map_err(|e| match e {
            nb::Error::Other(e) => e,
            nb::Error::WouldBlock => unreachable!(),
        })
    }

    /// Write bytes to an I2C device.
    pub fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), i2c::Error> {
        self.i2c.write(addr, bytes).map_err(|e| match e {
            nb::Error::Other(e) => e,
            nb::Error::WouldBlock => unreachable!(),
        })
    }
}
