//! Controller Area Network (CAN) abstraction layer.
//!
//! - `CanBus` wraps a HAL `can::Can` instance in `bxcan::Can`.
//! - Provides simple helpers for sending and receiving frames.

use core::convert::Infallible;
use nb::block;

use bxcan::{self, Data, Frame, OverrunError, StandardId, TransmitStatus};
use stm32f7xx_hal::can as hal_can;

/// Wrapper around a bxcan CAN instance built from a HAL CAN peripheral.
pub struct CanBus<I>
where
    hal_can::Can<I>: bxcan::Instance,
{
    can: bxcan::Can<hal_can::Can<I>>,
}

impl<I> CanBus<I>
where
    hal_can::Can<I>: bxcan::Instance,
{
    /// Create and enable a bxcan instance from a HAL CAN peripheral.
    ///
    /// * `hal_can` – the HAL CAN wrapper
    /// * `btr` – value for the CAN_BTR register (bit timing). Get this from the
    ///           reference manual or the bxcan timing tables.
    /// * `loopback` – enable internal loopback
    /// * `silent` – enable silent mode
    pub fn new(hal_can: hal_can::Can<I>, btr: u32, loopback: bool, silent: bool) -> Self {
        let can = bxcan::Can::builder(hal_can)
            .set_bit_timing(btr)
            .set_loopback(loopback)
            .set_silent(silent)
            .enable();

        Self { can }
    }

    /// Access the underlying bxcan instance for advanced configuration.
    pub fn inner(&mut self) -> &mut bxcan::Can<hal_can::Can<I>> {
        &mut self.can
    }

    /// Consume the wrapper and get back the underlying HAL CAN instance.
    pub fn free(self) -> hal_can::Can<I> {
        self.can.free()
    }

    /// Transmit a pre-built CAN frame.
    ///
    /// Returns the bxcan `TransmitStatus`. The error type is `Infallible`.
    pub fn transmit_frame(&mut self, frame: &Frame) -> Result<TransmitStatus, Infallible> {
        block!(self.can.transmit(frame))
    }

    /// Transmit a data frame with a standard 11-bit ID.
    ///
    /// `data` must be at most 8 bytes, otherwise this returns `None`.
    pub fn transmit_data(
        &mut self,
        id: StandardId,
        data: &[u8],
    ) -> Option<Result<TransmitStatus, Infallible>> {
        let data = Data::new(data)?;
        let frame = Frame::new_data(id, data);
        Some(block!(self.can.transmit(&frame)))
    }

    /// Blocking receive of a frame.
    ///
    /// This will block inside bxcan until a frame is available or an overrun is detected.
    pub fn receive(&mut self) -> Result<Frame, OverrunError> {
        block!(self.can.receive())
    }
}

/// Extra helpers for CAN instances that own filters (e.g., CAN1 on STM32F7).
impl<I> CanBus<I>
where
    hal_can::Can<I>: bxcan::Instance + bxcan::FilterOwner,
{
    /// Configure a single filter bank to accept all frames into FIFO0.
    pub fn configure_accept_all_filters(&mut self) {
        self.can.modify_filters().clear().enable_bank(
            0,
            bxcan::Fifo::Fifo0,
            bxcan::filter::Mask32::accept_all(),
        );
    }
}
