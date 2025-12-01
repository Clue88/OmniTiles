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
    /// Configure CAN1 and CAN2 filters so that both accept all frames on FIFO0.
    ///
    /// This must be called on CAN1 (the filter owner).
    pub fn configure_accept_all_filters_for_dual_can<I2>(&mut self, _can2: &mut hal_can::Can<I2>)
    where
        hal_can::Can<I2>: bxcan::Instance,
    {
        let regs = unsafe { &*stm32f7xx_hal::pac::CAN1::ptr() };

        // Enter filter init mode
        regs.fmr.modify(|_, w| w.finit().set_bit());

        // Set CAN2SB = split point. F7 has 28 filter banks (0-13 for CAN1, 14-27 for CAN2).
        regs.fmr.modify(|_, w| unsafe { w.can2sb().bits(14) });

        // Clear all filter banks (disable all)
        regs.fa1r.reset();
        regs.fm1r.reset();
        regs.fs1r.reset();
        regs.ffa1r.reset();

        // Set filter scale to 32-bit and mask mode for all banks
        regs.fs1r.modify(|_, w| unsafe { w.bits(0x0FFF_FFFF) });
        regs.fm1r.modify(|_, w| unsafe { w.bits(0x0000_0000) });

        // FIFO assignment -> FIFO0 for all banks
        regs.ffa1r.modify(|_, w| unsafe { w.bits(0x0000_0000) });

        // Set FR1/FR2 = 0 for accept-all
        // CAN1 bank 0
        regs.fb[0].fr1.write(|w| unsafe { w.bits(0) });
        regs.fb[0].fr2.write(|w| unsafe { w.bits(0) });

        // CAN2 bank 14
        regs.fb[14].fr1.write(|w| unsafe { w.bits(0) });
        regs.fb[14].fr2.write(|w| unsafe { w.bits(0) });

        // Activate bank 0 (CAN1) and bank 14 (CAN2)
        regs.fa1r
            .modify(|r, w| unsafe { w.bits(r.bits() | ((1 << 0) | (1 << 14))) });

        // Leave filter init mode (FINIT = 0)
        regs.fmr.modify(|_, w| w.finit().clear_bit());
    }
}
