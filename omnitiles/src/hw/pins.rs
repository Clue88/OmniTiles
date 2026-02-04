// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F777 MCU for OmniTiles.

use stm32f7xx_hal::{
    gpio::{
        gpioa, gpiob, gpioc, gpiod, gpioe, gpioh, Alternate, Analog, Floating, Input, Output,
        PushPull,
    },
    pac,
    prelude::*,
};

/// All board pins. Construct this once at startup using:
///
/// ```rust
/// let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOD, dp.GPIOE);
/// ```
pub struct BoardPins {
    pub leds: LedPins,
    pub usart1: Usart1Pins,
    pub spi4: Spi4Pins,
    pub m1: Motor1Pins,
    pub m2: Motor2Pins,
    pub encoder: EncoderPins,
    pub can1: Can1Pins,
    pub can2: Can2Pins,
}

pub struct LedPins {
    pub red: gpiod::PD8<Output<PushPull>>,
    pub yellow: gpiod::PD9<Output<PushPull>>,
    pub green: gpiod::PD10<Output<PushPull>>,
}

pub struct Usart1Pins {
    pub tx: gpioa::PA9<Alternate<7>>,
    pub rx: gpioa::PA10<Alternate<7>>,
}

/// SPI4 SCK/MISO/MOSI and CS
pub struct Spi4Pins {
    pub sck: gpioe::PE12<Alternate<5>>,
    pub miso: gpioe::PE13<Alternate<5>>,
    pub mosi: gpioe::PE14<Alternate<5>>,
    pub cs1: gpioe::PE4<Output<PushPull>>,
    pub cs2: gpioe::PE11<Output<PushPull>>,
}

/// TIM2/TIM3 Quadrature Encoder Inputs
pub struct EncoderPins {
    pub tim2_ch1: gpioa::PA0<Alternate<1>>,
    pub tim2_ch2: gpioa::PA1<Alternate<1>>,

    pub tim3_ch1: gpioa::PA6<Alternate<2>>,
    pub tim3_ch2: gpioa::PA7<Alternate<2>>,
}

/// Motor 1 control pins
pub struct Motor1Pins {
    pub in1: gpiod::PD12<Alternate<2>>, // TIM4_CH1 (PWM)
    pub in2: gpiod::PD13<Alternate<2>>, // TIM4_CH2 (PWM)
    pub nsleep: gpioa::PA4<Output<PushPull>>,
    pub disable: gpioa::PA3<Output<PushPull>>,
    pub nfault: gpioa::PA2<Input<Floating>>,
    pub iprop1: gpioc::PC4<Analog>, // ADC1_IN14
    pub iprop2: gpioc::PC5<Analog>, // ADC1_IN15
}

/// Motor 2 control pins
pub struct Motor2Pins {
    pub in1: gpiod::PD14<Alternate<2>>, // TIM4_CH3 (PWM)
    pub in2: gpiod::PD15<Alternate<2>>, // TIM4_CH4 (PWM)
    pub nsleep: gpiod::PD2<Output<PushPull>>,
    pub disable: gpiod::PD1<Output<PushPull>>,
    pub nfault: gpiod::PD0<Input<Floating>>,
    pub iprop1: gpioc::PC2<Analog>, // ADC1_IN12
    pub iprop2: gpioc::PC3<Analog>, // ADC1_IN13
}

/// CAN1 bus pins
pub struct Can1Pins {
    pub tx: gpioa::PA12<Alternate<9>>,
    pub rx: gpioa::PA11<Alternate<9>>,
}

/// CAN2 bus pins
pub struct Can2Pins {
    pub tx: gpiob::PB13<Alternate<9>>,
    pub rx: gpiob::PB12<Alternate<9>>,
}

impl BoardPins {
    /// Create all named pins from raw GPIO peripherals.
    pub fn new(
        gpioa: pac::GPIOA,
        gpiob: pac::GPIOB,
        gpioc: pac::GPIOC,
        gpiod: pac::GPIOD,
        gpioe: pac::GPIOE,
        gpioh: pac::GPIOH,
    ) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
        let gpioc = gpioc.split();
        let gpiod = gpiod.split();
        let gpioe = gpioe.split();
        let gpioh = gpioh.split();

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
                cs1: gpioe.pe4.into_push_pull_output(),
                cs2: gpioe.pe11.into_push_pull_output(),
            },

            encoder: EncoderPins {
                tim2_ch1: gpioa.pa0.into_alternate::<1>(),
                tim2_ch2: gpioa.pa1.into_alternate::<1>(),
                tim3_ch1: gpioa.pa6.into_alternate::<2>(),
                tim3_ch2: gpioa.pa7.into_alternate::<2>(),
            },

            m1: Motor1Pins {
                in1: gpiod.pd12.into_alternate::<2>(),
                in2: gpiod.pd13.into_alternate::<2>(),
                nsleep: gpioa.pa4.into_push_pull_output(),
                disable: gpioa.pa3.into_push_pull_output(),
                nfault: gpioa.pa2.into_floating_input(),
                iprop1: gpioc.pc4.into_analog(),
                iprop2: gpioc.pc5.into_analog(),
            },

            m2: Motor2Pins {
                in1: gpiod.pd14.into_alternate::<2>(),
                in2: gpiod.pd15.into_alternate::<2>(),
                nsleep: gpiod.pd2.into_push_pull_output(),
                disable: gpiod.pd1.into_push_pull_output(),
                nfault: gpiod.pd0.into_floating_input(),
                iprop1: gpioc.pc2.into_analog(),
                iprop2: gpioc.pc3.into_analog(),
            },

            can1: Can1Pins {
                tx: gpioa.pa12.into_alternate::<9>(),
                rx: gpioa.pa11.into_alternate::<9>().internal_pull_up(true),
            },

            can2: Can2Pins {
                tx: gpiob.pb13.into_alternate::<9>(),
                rx: gpiob.pb12.into_alternate::<9>().internal_pull_up(true),
            },
        }
    }
}
