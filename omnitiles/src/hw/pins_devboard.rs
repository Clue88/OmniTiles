// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F777 MCU Devboard for OmniTiles.

use stm32f7xx_hal::{
    gpio::{
        gpioa, gpiob, gpioc, gpiod, gpioe, Alternate, Analog, Floating, Input, OpenDrain, Output,
        PushPull,
    },
    pac,
    prelude::*,
};

/// All board pins. Construct this once at startup using:
///
/// ```rust
/// let pins = BoardPins::new(dp.GPIOA, dp.GPIOD, dp.GPIOE);
/// ```
pub struct BoardPins {
    pub leds: LedPins,
    pub usart1: Usart1Pins,
    pub spi4: Spi4Pins,
    pub m1: Motor1Pins,
    pub m2: Motor2Pins,
    pub i2c1: I2c1Pins,
    pub can1: Can1Pins,
}

pub struct LedPins {
    pub red: gpiod::PD8<Output<PushPull>>,
    pub yellow: gpiod::PD9<Output<PushPull>>,
    pub green: gpiod::PD10<Output<PushPull>>,
}

// USART1 TX/RX
pub struct Usart1Pins {
    pub tx: gpioa::PA9<Alternate<7>>,
    pub rx: gpioa::PA10<Alternate<7>>,
}

/// SPI4 SCK/MISO/MOSI and CS
pub struct Spi4Pins {
    pub sck: gpioe::PE12<Alternate<5>>,
    pub miso: gpioe::PE13<Alternate<5>>,
    pub mosi: gpioe::PE14<Alternate<5>>,
    pub cs: gpioe::PE4<Output<PushPull>>,
    pub drdy: gpioe::PE5<Input<Floating>>,
}

pub struct Motor1Pins {
    pub in1: gpioc::PC6<Alternate<2>>, // TIM3_CH1 (PWM)
    pub in2: gpioc::PC7<Alternate<2>>, // TIM3_CH2 (PWM)
    pub cs: gpiod::PD0<Output<PushPull>>,
    pub nsleep: gpiod::PD1<Output<PushPull>>,
    pub disable: gpiod::PD2<Output<PushPull>>,
    pub adc: gpiob::PB1<Analog>, // ADC1_IN9
}

pub struct Motor2Pins {
    pub in1: gpioc::PC8<Alternate<2>>, // TIM3_CH3 (PWM)
    pub in2: gpioc::PC9<Alternate<2>>, // TIM3_CH4 (PWM)
    pub cs: gpiod::PD3<Output<PushPull>>,
    pub nsleep: gpiod::PD4<Output<PushPull>>,
    pub disable: gpiod::PD5<Output<PushPull>>,
    pub adc: gpioc::PC2<Analog>, // ADC1_IN12
}

pub struct I2c1Pins {
    pub scl: gpiob::PB8<Alternate<4, OpenDrain>>,
    pub sda: gpiob::PB9<Alternate<4, OpenDrain>>,
}

/// CAN1 TX/RX
pub struct Can1Pins {
    pub tx: gpioa::PA12<Alternate<9>>,
    pub rx: gpioa::PA11<Alternate<9>>,
}

impl BoardPins {
    /// Create all named pins from raw GPIO peripherals.
    pub fn new(
        gpioa: pac::GPIOA,
        gpiob: pac::GPIOB,
        gpioc: pac::GPIOC,
        gpiod: pac::GPIOD,
        gpioe: pac::GPIOE,
    ) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
        let gpioc = gpioc.split();
        let gpiod = gpiod.split();
        let gpioe = gpioe.split();

        Self {
            leds: LedPins {
                red: gpiod.pd8.into_push_pull_output(),
                yellow: gpiod.pd9.into_push_pull_output(),
                green: gpiod.pd10.into_push_pull_output(),
            },

            usart1: Usart1Pins {
                tx: gpioa.pa9.into_alternate::<7>(),
                rx: gpioa.pa10.into_alternate::<7>(),
            },

            spi4: Spi4Pins {
                sck: gpioe.pe12.into_alternate::<5>(),
                miso: gpioe.pe13.into_alternate::<5>(),
                mosi: gpioe.pe14.into_alternate::<5>(),
                cs: gpioe.pe4.into_push_pull_output(),
                drdy: gpioe.pe5.into_floating_input(),
            },

            m1: Motor1Pins {
                in1: gpioc.pc6.into_alternate::<2>(),
                in2: gpioc.pc7.into_alternate::<2>(),
                cs: gpiod.pd0.into_push_pull_output(),
                nsleep: gpiod.pd1.into_push_pull_output(),
                disable: gpiod.pd2.into_push_pull_output(),
                adc: gpiob.pb1.into_analog(),
            },

            m2: Motor2Pins {
                in1: gpioc.pc8.into_alternate::<2>(),
                in2: gpioc.pc9.into_alternate::<2>(),
                cs: gpiod.pd3.into_push_pull_output(),
                nsleep: gpiod.pd4.into_push_pull_output(),
                disable: gpiod.pd5.into_push_pull_output(),
                adc: gpioc.pc2.into_analog(),
            },

            i2c1: I2c1Pins {
                scl: gpiob.pb8.into_alternate_open_drain::<4>(),
                sda: gpiob.pb9.into_alternate_open_drain::<4>(),
            },

            can1: Can1Pins {
                tx: gpioa.pa12.into_alternate::<9>(),
                rx: gpioa.pa11.into_alternate::<9>().internal_pull_up(true),
            },
        }
    }
}
