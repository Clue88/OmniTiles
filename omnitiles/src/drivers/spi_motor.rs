//! Motor abstraction for DRV8873 SPI driver with a TIM2 quadrature encoder.
//!
//! This module includes functions to drive the motor and read encoder values.

use crate::drivers::drv8873::{Diag, Drv8873, Fault};
use crate::hw::{Encoder, SpiBus};

use micromath::F32Ext;

use stm32f7xx_hal::{
    gpio::{self, Output, PushPull},
    pac, spi,
};

/// Logical drive direction / mode for the H-bridge.
#[derive(Copy, Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
    Brake,
    Coast,
}

/// Motor abstraction that combines a DRV8873 driver, four control pins, and a TIM2 encoder.
pub struct SpiMotor<
    const CS_P: char,
    const CS_N: u8,
    const IN1_P: char,
    const IN1_N: u8,
    const IN2_P: char,
    const IN2_N: u8,
    const SLP_P: char,
    const SLP_N: u8,
    const DIS_P: char,
    const DIS_N: u8,
> {
    drv: Drv8873<CS_P, CS_N>,
    enc: Encoder<pac::TIM2>,
    in1: gpio::Pin<IN1_P, IN1_N, Output<PushPull>>,
    in2: gpio::Pin<IN2_P, IN2_N, Output<PushPull>>,
    nsleep: gpio::Pin<SLP_P, SLP_N, Output<PushPull>>,
    disable: gpio::Pin<DIS_P, DIS_N, Output<PushPull>>,
    counts_per_rev: u32,
}

impl<
        const CS_P: char,
        const CS_N: u8,
        const IN1_P: char,
        const IN1_N: u8,
        const IN2_P: char,
        const IN2_N: u8,
        const SLP_P: char,
        const SLP_N: u8,
        const DIS_P: char,
        const DIS_N: u8,
    > SpiMotor<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N>
{
    /// Construct a new `SpiMotor`.
    ///
    /// `counts_per_rev` is the encoder resolution at the mechanical shaft (after any gear ratio).
    pub fn new<In1Mode, In2Mode, SlpMode, DisMode>(
        drv: Drv8873<CS_P, CS_N>,
        enc: Encoder<pac::TIM2>,
        in1: gpio::Pin<IN1_P, IN1_N, In1Mode>,
        in2: gpio::Pin<IN2_P, IN2_N, In2Mode>,
        nsleep: gpio::Pin<SLP_P, SLP_N, SlpMode>,
        disable: gpio::Pin<DIS_P, DIS_N, DisMode>,
        counts_per_rev: u32,
    ) -> Self {
        let mut in1 = in1.into_push_pull_output();
        let mut in2 = in2.into_push_pull_output();
        let mut nsleep = nsleep.into_push_pull_output();
        let mut disable = disable.into_push_pull_output();

        in1.set_low();
        in2.set_low();

        // Initialize as awake + disabled
        nsleep.set_high();
        disable.set_high();

        Self {
            drv,
            enc,
            in1,
            in2,
            nsleep,
            disable,
            counts_per_rev,
        }
    }

    /// Tear down this motor and return its constituent parts.
    pub fn free(
        self,
    ) -> (
        Drv8873<CS_P, CS_N>,
        Encoder<pac::TIM2>,
        gpio::Pin<IN1_P, IN1_N, Output<PushPull>>,
        gpio::Pin<IN2_P, IN2_N, Output<PushPull>>,
        gpio::Pin<SLP_P, SLP_N, Output<PushPull>>,
        gpio::Pin<DIS_P, DIS_N, Output<PushPull>>,
    ) {
        (
            self.drv,
            self.enc,
            self.in1,
            self.in2,
            self.nsleep,
            self.disable,
        )
    }

    /// Initialize and set base configuration for the DRV8873.
    pub fn init<I, PINS>(&mut self, spi_bus: &mut SpiBus<I, PINS>) -> Result<(), spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        let _fault = self.drv.read_fault(spi_bus)?;
        let _diag = self.drv.read_diag(spi_bus)?;

        Ok(())
    }

    /// Read the FAULT status register.
    #[inline]
    pub fn read_fault<I, PINS>(
        &mut self,
        spi_bus: &mut SpiBus<I, PINS>,
    ) -> Result<Fault, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        self.drv.read_fault(spi_bus)
    }

    /// Read the DIAG status register.
    #[inline]
    pub fn read_diag<I, PINS>(&mut self, spi_bus: &mut SpiBus<I, PINS>) -> Result<Diag, spi::Error>
    where
        I: spi::Instance,
        PINS: spi::Pins<I>,
    {
        self.drv.read_diag(spi_bus)
    }

    /// Access the underlying DRV8873 driver for advanced SPI control.
    #[inline]
    pub fn drv(&mut self) -> &mut Drv8873<CS_P, CS_N> {
        &mut self.drv
    }

    /// Put the driver into sleep mode.
    ///
    /// This shuts down most of the internal circuitry to reduce power consumption.
    #[inline]
    pub fn sleep(&mut self) {
        self.nsleep.set_low();
    }

    /// Wake the driver from sleep mode.
    #[inline]
    pub fn wake(&mut self) {
        self.nsleep.set_high();
    }

    /// Enable the motor and wake the driver if in sleep.
    #[inline]
    pub fn enable_outputs(&mut self) {
        self.wake();
        self.disable.set_low();
    }

    /// Disable the motor and coast.
    #[inline]
    pub fn disable_outputs(&mut self) {
        self.set_direction_pins(Direction::Coast);
        self.disable.set_high();
    }

    /// Configure IN1/IN2 pins for a given direction/mode.
    fn set_direction_pins(&mut self, dir: Direction) {
        match dir {
            Direction::Forward => {
                self.in1.set_high();
                self.in2.set_low();
            }
            Direction::Reverse => {
                self.in1.set_low();
                self.in2.set_high();
            }
            Direction::Brake => {
                self.in1.set_high();
                self.in2.set_high();
            }
            Direction::Coast => {
                self.in1.set_low();
                self.in2.set_low();
            }
        }
    }

    /// Drive the motor forward.
    #[inline]
    pub fn forward(&mut self) {
        self.set_direction_pins(Direction::Forward);
    }

    /// Drive the motor in reverse.
    #[inline]
    pub fn reverse(&mut self) {
        self.set_direction_pins(Direction::Reverse);
    }

    /// Apply brakes.
    #[inline]
    pub fn brake(&mut self) {
        self.set_direction_pins(Direction::Brake);
    }

    /// Coast (this sets outputs to HiZ).
    #[inline]
    pub fn coast(&mut self) {
        self.set_direction_pins(Direction::Coast);
    }

    /// Raw encoder position in ticks (signed).
    #[inline]
    pub fn position_ticks(&self) -> i32 {
        self.enc.position()
    }

    /// Encoder position converted to revolutions.
    #[inline]
    pub fn position_revs(&self) -> f32 {
        self.enc.position() as f32 / self.counts_per_rev as f32
    }

    /// Reset the encoder position to zero.
    #[inline]
    pub fn zero(&mut self) {
        self.enc.reset();
    }

    /// Convert a number of revolutions into encoder ticks.
    #[inline]
    pub fn ticks_for_revs(&self, revs: f32) -> i32 {
        (revs * self.counts_per_rev as f32).round() as i32
    }

    /// Compute a target tick position for a relative move from the current position.
    #[inline]
    pub fn target_for_delta_ticks(&self, delta_ticks: i32) -> i32 {
        self.position_ticks().wrapping_add(delta_ticks)
    }

    /// Compute a target tick position for a relative move in revolutions.
    #[inline]
    pub fn target_for_delta_revs(&self, delta_revs: f32) -> i32 {
        let delta_ticks = self.ticks_for_revs(delta_revs);
        self.target_for_delta_ticks(delta_ticks)
    }

    /// Expose the underlying encoder.
    #[inline]
    pub fn encoder(&self) -> &Encoder<pac::TIM2> {
        &self.enc
    }

    /// Mutable access to the encoder.
    #[inline]
    pub fn encoder_mut(&mut self) -> &mut Encoder<pac::TIM2> {
        &mut self.enc
    }

    /// Apply PID output.
    pub fn apply_pid_output(&mut self, u: f32) {
        if u > 0.0 {
            self.forward();
        } else if u < 0.0 {
            self.reverse();
        } else {
            self.coast();
        }
    }
}
