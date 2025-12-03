//! Quadrature encoder support via STM32F7 timers in encoder mode.
//!
//! This module configures TIM2 (32-bit) and TIM3 (16-bit) reigsters for encoder mode and provides
//! simple accessors.

use stm32f7xx_hal::pac;

pub struct Encoder<TIM> {
    tim: TIM,
}

impl<TIM> Encoder<TIM> {
    /// Consume the wrapper and return the underlying timer peripheral.
    #[inline]
    pub fn free(self) -> TIM {
        self.tim
    }
}

impl Encoder<pac::TIM2> {
    /// Configure TIM2 as a quadrature encoder with full 32-bit range.
    pub fn tim2(tim2: pac::TIM2) -> Self {
        let tim = tim2;

        // Disable counter while configuring
        tim.cr1.modify(|_, w| w.cen().clear_bit());

        // Auto-reload: max 32-bit
        tim.arr.write(|w| w.bits(0xFFFF_FFFF));

        // Slave mode: encoder mode 3 (count on both TI1 and TI2)
        tim.smcr.modify(|_, w| w.sms().bits(0b011));

        // Configure CH1/CH2 as inputs from TI1/TI2
        tim.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti2());

        // Polarity and enable for both channels.
        tim.ccer.modify(|_, w| {
            w.cc1p()
                .clear_bit()
                .cc2p()
                .clear_bit()
                .cc1e()
                .set_bit()
                .cc2e()
                .set_bit()
        });

        // Reset the counter
        tim.cnt.write(|w| w.bits(0));

        // Enable the counter
        tim.cr1.modify(|_, w| w.cen().set_bit());

        Self { tim }
    }

    /// Read the raw 32-bit counter value.
    #[inline]
    pub fn raw(&self) -> u32 {
        self.tim.cnt.read().cnt().bits()
    }

    /// Interpret the counter as a signed 32-bit position.
    #[inline]
    pub fn position(&self) -> i32 {
        self.raw() as i32
    }

    /// Reset the encoder position to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.tim.cnt.write(|w| w.bits(0));
    }
}

impl Encoder<pac::TIM3> {
    /// Configure TIM3 as a quadrature encoder with full 16-bit range.
    pub fn tim3(tim3: pac::TIM3) -> Self {
        let tim = tim3;

        // Disable counter while configuring
        tim.cr1.modify(|_, w| w.cen().clear_bit());

        // Auto-reload: max 16-bit
        tim.arr.write(|w| unsafe { w.bits(0xFFFF) });

        // Slave mode: encoder mode 3 (count on both TI1 and TI2)
        tim.smcr.modify(|_, w| w.sms().bits(0b011));

        // Configure CH1/CH2 as inputs from TI1/TI2
        tim.ccmr1_input().modify(|_, w| w.cc1s().ti1().cc2s().ti2());

        // Polarity and enable for both channels.
        tim.ccer.modify(|_, w| {
            w.cc1p()
                .clear_bit()
                .cc2p()
                .clear_bit()
                .cc1e()
                .set_bit()
                .cc2e()
                .set_bit()
        });

        // Reset counter
        tim.cnt.write(|w| unsafe { w.bits(0) });

        // Enable counter
        tim.cr1.modify(|_, w| w.cen().set_bit());

        Self { tim }
    }

    /// Read the raw 16-bit counter value.
    #[inline]
    pub fn raw(&self) -> u16 {
        self.tim.cnt.read().cnt().bits()
    }

    /// Interpret the counter as a signed 16-bit position.
    #[inline]
    pub fn position(&self) -> i16 {
        self.raw() as i16
    }

    /// Reset the encoder position to zero.
    #[inline]
    pub fn reset(&mut self) {
        self.tim.cnt.write(|w| unsafe { w.bits(0) });
    }
}
