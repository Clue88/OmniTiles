//! High-level CAN motor control for the SteadyWin GIM6010-48 Planetary Reducer Motor with a GDZ468
//! driver.
//!
//! This module wraps the driver's custom CAN protocol for a single motor.

use crate::hw::CanBus;

use bxcan::{Frame, Id, OverrunError, StandardId};
use core::convert::TryInto;
use micromath::F32Ext;

/// Error type for `CanMotor` operations.
#[derive(Debug)]
pub enum Error {
    /// Payload too long for a single CAN data frame (max 8 bytes total).
    PayloadTooLong,
    /// TX mailbox or transmit failure.
    TxMailbox,
    /// Receive-side CAN error.
    Rx(OverrunError),
    /// Response frame contained no data bytes.
    NoData,
    /// Response had a different command code than expected.
    UnexpectedCommand(u8),
}

impl From<OverrunError> for Error {
    fn from(e: OverrunError) -> Self {
        Error::Rx(e)
    }
}

/// High-level CAN motor for a single driver instance, parameterized by logical device address.
///
/// `DEV_ADDR` is the protocol device address (`Dev_addr`), in the range 1 to 254 inclusive. The
/// default `DEV_ADDR` is 0x01.
///
/// The driver will:
///   - transmit commands with `StdID = 0x100 | DEV_ADDR`
///   - expect responses from `StdID = DEV_ADDR`
pub struct CanMotor<const DEV_ADDR: u16>;

impl<const DEV_ADDR: u16> CanMotor<DEV_ADDR> {
    /// Create a new handle for this motor address.
    ///
    /// This is a zero-sized type; all state lives on the driver itself.
    #[inline]
    pub fn new() -> Self {
        Self
    }

    /// Host -> motor StdID (11-bit) used for commands.
    #[inline]
    fn host_id() -> StandardId {
        StandardId::new((0x100 | (DEV_ADDR & 0x7FF)) as u16).unwrap()
    }

    /// Motor -> host StdID (11-bit) used for responses.
    #[inline]
    fn dev_id() -> StandardId {
        StandardId::new(DEV_ADDR & 0x7FF).unwrap()
    }

    /// Send a command an optionally wait for its response.
    ///
    /// - `cmd` is the command code (e.g., 0xA2 for read speed).
    /// - `payload` is any extra bytes following the command code.
    /// - If `wait_reply` is true, this will block until a matching response frame (same device ID
    ///   and command code) is received.
    ///
    /// Returns:
    ///   - `Ok(Some(data))` when `wait_reply` is true and a response was received.
    ///   - `Ok(None)` when `wait_reply` is false.
    fn request_response<I>(
        &mut self,
        bus: &mut CanBus<I>,
        cmd: u8,
        payload: &[u8],
        wait_reply: bool,
    ) -> Result<Option<[u8; 8]>, Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        // Total payload including command must be <= 8 bytes
        if payload.len() > 7 {
            return Err(Error::PayloadTooLong);
        }

        // Build TX buffer
        let mut buf = [0u8; 8];
        buf[0] = cmd;
        let dlc = 1 + payload.len();
        buf[1..dlc].copy_from_slice(payload);

        // Transmit
        let tx_id = Self::host_id();
        let tx_result = bus
            .transmit_data(tx_id, &buf[..dlc])
            .ok_or(Error::PayloadTooLong)?;

        match tx_result {
            Ok(_status) => {}
            Err(_) => return Err(Error::TxMailbox),
        }

        if !wait_reply {
            return Ok(None);
        }

        // Wait for matching response
        loop {
            let frame: Frame = bus.receive()?;

            let id = match frame.id() {
                Id::Standard(id) => id,
                Id::Extended(_) => continue, // ignore extended frames
            };
            if id != Self::dev_id() {
                continue; // some other device's response
            }

            let data = match frame.data() {
                Some(d) if d.len() > 0 => d,
                _ => return Err(Error::NoData),
            };

            let resp_cmd = data[0];
            if resp_cmd != cmd {
                continue; // Ignore if different command
            }

            let mut out = [0u8; 8];
            let len = data.len().min(8);
            out[..len].copy_from_slice(&data[..len]);
            return Ok(Some(out));
        }
    }

    /// Clear any latched faults.
    pub fn clear_faults<I>(&mut self, bus: &mut CanBus<I>) -> Result<(), Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        let _ = self.request_response(bus, 0xAF, &[], true)?;
        Ok(())
    }

    /// Turn off motor output. The motor enters a free state and becomes uncontrollable (this is the
    /// state the motor enters after power-on).
    pub fn disable_output<I>(&mut self, bus: &mut CanBus<I>) -> Result<(), Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        let _ = self.request_response(bus, 0xCF, &[], false)?;
        Ok(())
    }

    /// Command the motor in speed control mode with a setpoint in rpm.
    ///
    /// - `rpm` is signed (negative values indicate reverse direction).
    /// - Resolution is 0.01 rpm
    pub fn set_speed_rpm<I>(&mut self, bus: &mut CanBus<I>, rpm: f32) -> Result<(), Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        // Convert to protocol units: little-endian signed 32-bit int, 0.01 rpm
        let scaled: i32 = (rpm * 100.0).round() as i32;
        let bytes = scaled.to_le_bytes();

        let _ = self.request_response(bus, 0xC1, &bytes, false)?;
        Ok(())
    }

    /// Read back the real-time motor speed in rpm.
    pub fn read_speed_rpm<I>(&mut self, bus: &mut CanBus<I>) -> Result<f32, Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        let resp = self
            .request_response(bus, 0xA2, &[], true)?
            .ok_or(Error::NoData)?;

        if resp[0] != 0xA2 {
            return Err(Error::UnexpectedCommand(resp[0]));
        }

        let speed_bytes: [u8; 4] = resp[1..5].try_into().expect("slice with exact length");
        let raw = i32::from_le_bytes(speed_bytes);
        let rpm = raw as f32 / 100.0;
        Ok(rpm)
    }

    // Read the raw status frame.
    pub fn read_status_frame<I>(&mut self, bus: &mut CanBus<I>) -> Result<[u8; 8], Error>
    where
        stm32f7xx_hal::can::Can<I>: bxcan::Instance,
    {
        let resp = self
            .request_response(bus, 0xAE, &[], true)?
            .ok_or(Error::NoData)?;

        if resp[0] != 0xAE {
            return Err(Error::UnexpectedCommand(resp[0]));
        }

        Ok(resp)
    }
}
