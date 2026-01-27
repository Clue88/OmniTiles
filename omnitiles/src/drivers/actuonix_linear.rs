// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Generic driver for Actuonix 16-series (P16, T16) Linear Actuators with potentiometer control.
//!
//! Wraps a DRV8873 H-Bridge for motor control and an ADC channel for position feedback.
//!
//! Wiring (Option -P):
//! - Pin 1 (Orange): Potentiometer Ground
//! - Pin 2 (Purple): Potentiometer Wiper (ADC Input)
//! - Pin 3 (Red):    Motor Terminal A (+)
//! - Pin 4 (Black):  Motor Terminal B (-)
//! - Pin 5 (Yellow): Potentiometer Reference (3.3V)

use crate::drivers::drv8873::{Drv8873, Fault};
use crate::hw::SpiBus;

use stm32f7xx_hal::{
    gpio::{self, Output, PushPull},
    spi,
};

/// Logical drive direction for the linear actuator.
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Direction {
    Extend,
    Retract,
    Brake,
    Coast,
}

/// Generic driver for Actuonix linear actuators (P16, T16).
///
/// `ReadPos` is a closure that returns the raw 12-bit ADC reading (0..4095).
pub struct ActuonixLinear<
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
    ReadPos,
> {
    drv: Drv8873<CS_P, CS_N>,
    in1: gpio::Pin<IN1_P, IN1_N, Output<PushPull>>,
    in2: gpio::Pin<IN2_P, IN2_N, Output<PushPull>>,
    nsleep: gpio::Pin<SLP_P, SLP_N, Output<PushPull>>,
    disable: gpio::Pin<DIS_P, DIS_N, Output<PushPull>>,
    read_position: ReadPos,
    stroke_len_mm: f32,
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
        ReadPos,
    > ActuonixLinear<CS_P, CS_N, IN1_P, IN1_N, IN2_P, IN2_N, SLP_P, SLP_N, DIS_P, DIS_N, ReadPos>
where
    ReadPos: FnMut() -> u16,
{
    /// Construct a new P16 driver.
    pub fn new<In1Mode, In2Mode, SlpMode, DisMode>(
        drv: Drv8873<CS_P, CS_N>,
        in1: gpio::Pin<IN1_P, IN1_N, In1Mode>,
        in2: gpio::Pin<IN2_P, IN2_N, In2Mode>,
        nsleep: gpio::Pin<SLP_P, SLP_N, SlpMode>,
        disable: gpio::Pin<DIS_P, DIS_N, DisMode>,
        read_position: ReadPos,
        stroke_len_mm: f32,
    ) -> Self {
        let mut in1 = in1.into_push_pull_output();
        let mut in2 = in2.into_push_pull_output();
        let mut nsleep = nsleep.into_push_pull_output();
        let mut disable = disable.into_push_pull_output();

        // Default: Coast, Awake, Enabled
        in1.set_low();
        in2.set_low();
        nsleep.set_high();
        disable.set_low();

        Self {
            drv,
            in1,
            in2,
            nsleep,
            disable,
            read_position,
            stroke_len_mm,
        }
    }

    /// Set the manual drive direction.
    pub fn set_direction(&mut self, dir: Direction) {
        match dir {
            Direction::Extend => {
                // Polarity: IN1 High, IN2 Low
                self.in1.set_high();
                self.in2.set_low();
            }
            Direction::Retract => {
                // Polarity: IN1 Low, IN2 High
                self.in1.set_low();
                self.in2.set_high();
            }
            Direction::Brake => {
                // Short motor terminals: IN1 High, IN2 High
                self.in1.set_high();
                self.in2.set_high();
            }
            Direction::Coast => {
                // Open motor terminals: IN1 Low, IN2 Low
                self.in1.set_low();
                self.in2.set_low();
            }
        }
    }

    /// Extend the actuator.
    #[inline]
    pub fn extend(&mut self) {
        self.set_direction(Direction::Extend);
    }

    /// Retract the actuator.
    #[inline]
    pub fn retract(&mut self) {
        self.set_direction(Direction::Retract);
    }

    /// Brake (stops quickly).
    #[inline]
    pub fn brake(&mut self) {
        self.set_direction(Direction::Brake);
    }

    /// Coast (stops slowly).
    #[inline]
    pub fn coast(&mut self) {
        self.set_direction(Direction::Coast);
    }

    /// Read raw 12-bit ADC value (0-4095).
    #[inline]
    pub fn position_raw(&mut self) -> u16 {
        (self.read_position)()
    }

    /// Read position as a fraction (0.0 = Retracted, 1.0 = Extended).
    pub fn position_percent(&mut self) -> f32 {
        let raw = self.position_raw();
        // 12-bit ADC = 4095 max.
        // We assume 0V = 0 ticks, 3.3V = 4095 ticks.
        (raw as f32) / 4095.0
    }

    /// Read position in millimeters.
    pub fn position_mm(&mut self) -> f32 {
        self.position_percent() * self.stroke_len_mm
    }

    /// Access the inner DRV8873 for fault reading.
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
        self.coast();
        self.disable.set_high();
    }

    /// Read the FAULT status register from the DRV8873.
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
}
