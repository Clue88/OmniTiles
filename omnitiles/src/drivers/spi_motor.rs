//! Motor abstraction for DRV8873 SPI driver with a TIM2 quadrature encoder.
//!
//! This module connects `Drv8873` and `Encoder<TIM2>`. The SPI bus is not owned here and must be
//! passed in as `&mut SpiBus` so that multiple devices can share one bus safely.

use crate::drivers::drv8873::{reg, Drv8873, Response, Status};
use crate::hw::{Encoder, SpiBus};
use stm32f7xx_hal::{pac, spi};

/// Logical drive direction for the motor.
#[derive(Copy, Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
    Brake,
    Coast,
}

/// High-level motor that combines a DRV8873 driver IC (SPI + CS) and a TIM2 32-bit encoder.
pub struct SpiMotor<const P: char, const N: u8> {
    drv: Drv8873<P, N>,
    enc: Encoder<pac::TIM2>,
    counts_per_rev: u32,
}

impl<const P: char, const N: u8> SpiMotor<P, N> {
    /// Create a new motor from its DRV8873 driver and TIM2 encoder.
    ///
    /// `counts_per_rev` is the encoder resolution after gear ratio: i.e., ticks at the shaft per
    /// one mechanical revolution.
    pub fn new(drv: Drv8873<P, N>, enc: Encoder<pac::TIM2>, counts_per_rev: u32) -> Self {
        Self {
            drv,
            enc,
            counts_per_rev,
        }
    }

    /// Tear down this motor and return its constituent parts.
    pub fn free(self) -> (Drv8873<P, N>, Encoder<pac::TIM2>) {
        (self.drv, self.enc)
    }

    /// Configure the DRV8873 into a known safe operating mode.
    /// - Set the control mode
    /// - Configure current limits, OCP behavior, etc.
    /// - Clear latched faults as appropriate.
    pub fn init<I, PINS>(&mut self, spi_bus: &mut SpiBus<I, PINS>) -> Result<(), spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        // Example:
        //   - configure IC1/IC2/IC3/IC4 with your selected mode.
        //
        // let ic1_val = ...; // TODO: derive from DRV8873-Q1 datasheet
        // let ic2_val = ...;
        // let ic3_val = ...;
        // let ic4_val = ...;
        //
        // let _ = self.drv.write_reg(spi_bus, reg::IC1, ic1_val)?;
        // let _ = self.drv.write_reg(spi_bus, reg::IC2, ic2_val)?;
        // let _ = self.drv.write_reg(spi_bus, reg::IC3, ic3_val)?;
        // let _ = self.drv.write_reg(spi_bus, reg::IC4, ic4_val)?;

        // For now, just read back FAULT/DIAG as a sanity check.
        let _ = self.drv.read_fault(spi_bus)?;
        let _ = self.drv.read_diag(spi_bus)?;

        Ok(())
    }

    /// Read the current status flags (OTW, UVLO, OCP, etc.).
    pub fn read_status<I, PINS>(
        &mut self,
        spi_bus: &mut SpiBus<I, PINS>,
    ) -> Result<Status, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        let resp = self.drv.read_fault(spi_bus)?;
        Ok(resp.status)
    }

    /// Enable the motor output stage.
    /// - Clear disable bits
    /// - Configure bridge mode
    /// - Ensure the driver is awake and ready.
    pub fn enable<I, PINS>(&mut self, spi_bus: &mut SpiBus<I, PINS>) -> Result<(), spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        // TODO: Implement by updating ICx registers (e.g., set EN bits).
        // let mut ic1 = self.drv.read_ic1(spi_bus)?.data;
        // ic1 |= ...; // set enable bit(s)
        // let _ = self.drv.write_ic1(spi_bus, ic1)?;
        let _ = spi_bus; // silence unused warning for now
        Ok(())
    }

    /// Disable the motor output stage (coast or brake, depending on config).
    pub fn disable<I, PINS>(&mut self, spi_bus: &mut SpiBus<I, PINS>) -> Result<(), spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        // TODO: Implement by clearing EN bits or forcing a safe state.
        let _ = spi_bus;
        Ok(())
    }

    /// Command the motor with a desired direction and duty cycle.
    ///
    /// `duty` is a normalized value in [0.0, 1.0], where:
    ///   - 0.0 -> no drive
    ///   - 1.0 -> maximum allowed drive (as configured in IC registers)
    ///
    /// Internally, this should:
    ///   - map `Direction` to DRV8873 mode bits (e.g., PH/EN or in1/in2 states),
    ///   - quantize `duty` into whatever resolution the DRV8873 expects.
    pub fn set_drive<I, PINS>(
        &mut self,
        spi_bus: &mut SpiBus<I, PINS>,
        dir: Direction,
        duty: f32,
    ) -> Result<(), spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        // Clamp duty to [0.0, 1.0]
        let duty = if duty < 0.0 {
            0.0
        } else if duty > 1.0 {
            1.0
        } else {
            duty
        };

        // TODO:
        //  - turn `dir` + `duty` into appropriate ICx values,
        //  - e.g., set PH bit, EN PWM amplitude, mode selection, etc.

        let _ = (spi_bus, dir, duty); // placeholder

        Ok(())
    }

    /// Raw encoder position in ticks, centered around 0.
    #[inline]
    pub fn position_ticks(&self) -> i32 {
        self.enc.position()
    }

    /// Encoder position converted to revolutions (float).
    #[inline]
    pub fn position_revs(&self) -> f32 {
        self.enc.position() as f32 / self.counts_per_rev as f32
    }

    /// Reset the encoder position to zero.
    #[inline]
    pub fn zero(&mut self) {
        self.enc.reset();
    }

    /// Expose the underlying encoder if needed for advanced usage.
    #[inline]
    pub fn encoder(&self) -> &Encoder<pac::TIM2> {
        &self.enc
    }

    /// Mutable access to the encoder.
    #[inline]
    pub fn encoder_mut(&mut self) -> &mut Encoder<pac::TIM2> {
        &mut self.enc
    }
}
