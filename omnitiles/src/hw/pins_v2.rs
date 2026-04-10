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

/// All board pins. Construct this once at startup.
pub struct BoardPins {
    pub leds: LedPins,
    pub usart1: Usart1Pins,
    pub spi4: Spi4Pins,
    pub m1: Motor1Pins,
    pub m2: Motor2Pins,
    pub i2c1: I2c1Pins,
    #[cfg(feature = "mobile-base")]
    pub wheels: WheelPins,
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
    pub miso: gpioe::PE5<Alternate<5>>,
    pub mosi: gpioe::PE14<Alternate<5>>,
    pub cs1: gpioe::PE4<Output<PushPull>>,
    pub drdy: gpioe::PE11<Input<Floating>>,
    pub cs2: gpioe::PE3<Output<PushPull>>,
}

pub struct Motor1Pins {
    pub in1: gpioc::PC6<Alternate<2>>, // TIM3_CH1 (PWM)
    pub in2: gpioc::PC7<Alternate<2>>, // TIM3_CH2 (PWM)
    pub nsleep: gpiod::PD2<Output<PushPull>>,
    pub disable: gpiod::PD1<Output<PushPull>>,
    pub adc1: gpiob::PB0<Analog>, // ADC1_IN8
    pub adc2: gpiob::PB1<Analog>, // ADC1_IN9
    pub adc3: gpioc::PC0<Analog>, // ADC1_IN10
    pub adc4: gpioc::PC1<Analog>, // ADC1_IN11
}

pub struct Motor2Pins {
    pub in1: gpioc::PC8<Alternate<2>>, // TIM3_CH3 (PWM)
    pub in2: gpioc::PC9<Alternate<2>>, // TIM3_CH4 (PWM)
    pub nsleep: gpiod::PD4<Output<PushPull>>,
    pub disable: gpiod::PD5<Output<PushPull>>,
    pub adc1: gpioc::PC2<Analog>, // ADC1_IN12
    pub adc2: gpioc::PC3<Analog>, // ADC1_IN13
}

pub struct I2c1Pins {
    pub scl: gpiob::PB6<Alternate<4, OpenDrain>>,
    pub sda: gpiob::PB9<Alternate<4, OpenDrain>>,
}

#[cfg(feature = "mobile-base")]
pub struct WheelPins {
    pub fl_pwm: gpiod::PD12<Alternate<2>>, // TIM4_CH1
    pub fl_in1: gpiod::PD3<Output<PushPull>>,
    pub fl_in2: gpiod::PD6<Output<PushPull>>,
    pub fr_pwm: gpiod::PD13<Alternate<2>>, // TIM4_CH2
    pub fr_in1: gpiod::PD7<Output<PushPull>>,
    pub fr_in2: gpiod::PD11<Output<PushPull>>,
    pub bl_pwm: gpiod::PD14<Alternate<2>>, // TIM4_CH3
    pub bl_in1: gpioe::PE0<Output<PushPull>>,
    pub bl_in2: gpioe::PE1<Output<PushPull>>,
    pub br_pwm: gpiod::PD15<Alternate<2>>, // TIM4_CH4
    pub br_in1: gpioe::PE2<Output<PushPull>>,
    pub br_in2: gpioe::PE6<Output<PushPull>>,
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
                miso: gpioe.pe5.into_alternate::<5>(),
                mosi: gpioe.pe14.into_alternate::<5>(),
                cs1: gpioe.pe4.into_push_pull_output(),
                drdy: gpioe.pe11.into_floating_input(),
                cs2: gpioe.pe3.into_push_pull_output(),
            },

            m1: Motor1Pins {
                in1: gpioc.pc6.into_alternate::<2>(),
                in2: gpioc.pc7.into_alternate::<2>(),
                nsleep: gpiod.pd2.into_push_pull_output(),
                disable: gpiod.pd1.into_push_pull_output(),
                adc1: gpiob.pb0.into_analog(),
                adc2: gpiob.pb1.into_analog(),
                adc3: gpioc.pc0.into_analog(), // Inverted
                adc4: gpioc.pc1.into_analog(), // Inverted
            },

            m2: Motor2Pins {
                in1: gpioc.pc8.into_alternate::<2>(),
                in2: gpioc.pc9.into_alternate::<2>(),
                nsleep: gpiod.pd4.into_push_pull_output(),
                disable: gpiod.pd5.into_push_pull_output(),
                adc1: gpioc.pc2.into_analog(),
                adc2: gpioc.pc3.into_analog(),
            },

            i2c1: I2c1Pins {
                scl: gpiob
                    .pb6
                    .into_alternate::<4>()
                    .internal_pull_up(true)
                    .set_open_drain(),
                sda: gpiob
                    .pb9
                    .into_alternate::<4>()
                    .internal_pull_up(true)
                    .set_open_drain(),
            },

            #[cfg(feature = "mobile-base")]
            wheels: WheelPins {
                fl_pwm: gpiod.pd12.into_alternate::<2>(),
                fl_in1: gpiod.pd3.into_push_pull_output(),
                fl_in2: gpiod.pd6.into_push_pull_output(),
                fr_pwm: gpiod.pd13.into_alternate::<2>(),
                fr_in1: gpiod.pd7.into_push_pull_output(),
                fr_in2: gpiod.pd11.into_push_pull_output(),
                bl_pwm: gpiod.pd14.into_alternate::<2>(),
                bl_in1: gpioe.pe0.into_push_pull_output(),
                bl_in2: gpioe.pe1.into_push_pull_output(),
                br_pwm: gpiod.pd15.into_alternate::<2>(),
                br_in1: gpioe.pe2.into_push_pull_output(),
                br_in2: gpioe.pe6.into_push_pull_output(),
            },
        }
    }
}
