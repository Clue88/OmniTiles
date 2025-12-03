//! Basic ADC support for STM32F7 using direct PAC register access.
//!
//! Thin wrapper around ADC1/ADC2/ADC3 with blocking single-channel reads.
//!
//! Example:
//! ```no_run
//! let adc1 = Adc::adc1(dp.ADC1, &rcc);
//! let value = adc1.read(3);
//! ```

use core::cell::RefCell;

use stm32f7xx_hal::pac;

/// Generic ADC wrapper over a PAC ADCx peripheral.
pub struct Adc<ADC> {
    adc: ADC,
}

impl<ADC> Adc<ADC> {
    #[inline]
    pub fn free(self) -> ADC {
        self.adc
    }
}

/// Trait for reading a single channel from an ADC peripheral.
pub trait AdcRead {
    fn read_channel(&mut self, ch: u8) -> u16;
}

fn configure_common() {
    let common = unsafe { &*pac::ADC_COMMON::ptr() };

    // ADC prescaler: PCLK2 / 4
    common.ccr.modify(|_, w| w.adcpre().div4());
}

fn init_basic_adc(adc: &pac::adc1::RegisterBlock) {
    // Power off to configure
    adc.cr2.modify(|_, w| w.adon().clear_bit());

    // 12-bit, right-aligned, software trigger
    adc.cr1.modify(|_, w| w.res().bits(0b00));
    adc.cr2.modify(|_, w| {
        w.cont().clear_bit();
        w.align().right();
        w.exten().disabled();
        w
    });

    // Default minimal sample times
    adc.smpr2.modify(|_, w| unsafe { w.bits(0) });

    // Power on
    adc.cr2.modify(|_, w| w.adon().set_bit());
}

impl Adc<pac::ADC1> {
    /// Create and initialize ADC1.
    pub fn adc1(adc1: pac::ADC1) -> Self {
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.adc1en().set_bit());

        configure_common();
        init_basic_adc(&adc1);

        Self { adc: adc1 }
    }
}

impl Adc<pac::ADC2> {
    /// Create and initialize ADC2.
    pub fn adc2(adc2: pac::ADC2) -> Self {
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.adc2en().set_bit());

        configure_common();
        init_basic_adc(&adc2);

        Self { adc: adc2 }
    }
}

impl Adc<pac::ADC3> {
    /// Create and initialize ADC3.
    pub fn adc3(adc3: pac::ADC3) -> Self {
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb2enr.modify(|_, w| w.adc3en().set_bit());

        configure_common();
        init_basic_adc(&adc3);

        Self { adc: adc3 }
    }
}

/// Read a single channel from the given ADC peripheral.
fn read_channel(adc: &pac::adc1::RegisterBlock, channel: u8) -> u16 {
    // Configure long sample time for channel stability
    if channel <= 9 {
        adc.smpr2.modify(|_, w| match channel {
            0 => w.smp0().bits(0b111),
            1 => w.smp1().bits(0b111),
            2 => w.smp2().bits(0b111),
            3 => w.smp3().bits(0b111),
            4 => w.smp4().bits(0b111),
            5 => w.smp5().bits(0b111),
            6 => w.smp6().bits(0b111),
            7 => w.smp7().bits(0b111),
            8 => w.smp8().bits(0b111),
            9 => w.smp9().bits(0b111),
            _ => unreachable!(),
        });
    }

    // Sequence length = 1 conversion
    adc.sqr1.modify(|_, w| w.l().bits(0));

    // Set channel
    adc.sqr3
        .modify(|_, w| unsafe { w.sq1().bits(channel & 0x1F) });

    // Start
    adc.cr2.modify(|_, w| w.swstart().set_bit());

    // Wait for completion
    while adc.sr.read().eoc().bit_is_clear() {}

    adc.dr.read().data().bits() as u16
}

impl Adc<pac::ADC1> {
    /// Read a single channel.
    #[inline]
    pub fn read(&self, channel: u8) -> u16 {
        read_channel(&self.adc, channel)
    }
}

impl Adc<pac::ADC2> {
    /// Read a single channel.
    #[inline]
    pub fn read(&self, channel: u8) -> u16 {
        read_channel(&self.adc, channel)
    }
}

impl Adc<pac::ADC3> {
    /// Read a single channel.
    #[inline]
    pub fn read(&self, channel: u8) -> u16 {
        read_channel(&self.adc, channel)
    }
}

impl AdcRead for Adc<pac::ADC1> {
    fn read_channel(&mut self, ch: u8) -> u16 {
        self.read(ch)
    }
}

impl AdcRead for Adc<pac::ADC2> {
    fn read_channel(&mut self, ch: u8) -> u16 {
        self.read(ch)
    }
}

impl AdcRead for Adc<pac::ADC3> {
    fn read_channel(&mut self, ch: u8) -> u16 {
        self.read(ch)
    }
}

impl<ADC> Adc<ADC>
where
    Adc<ADC>: AdcRead,
{
    /// Create a closure that reads the given channel from the ADC reference.
    pub fn make_reader<'a>(adc_ref: &'a RefCell<Self>, channel: u8) -> impl FnMut() -> u16 + 'a {
        move || adc_ref.borrow_mut().read_channel(channel)
    }
}
