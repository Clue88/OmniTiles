// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F767ZI devboard.

use stm32f7xx_hal::{
    gpio::{
        gpioa, gpiob, gpioc, gpiod, Alternate, Analog, Floating, Input, OpenDrain, Output, PushPull,
    },
    pac,
    prelude::*,
};
#[cfg(feature = "mobile-base")]
use stm32f7xx_hal::gpio::gpioe;

pub struct BoardPins {
    pub leds: Leds,
    pub usart3: Usart3Pins,
    pub spi1: Spi1Pins,
    pub m1: Motor1Pins,
    pub m2: Motor2Pins,
    pub i2c1: I2c1Pins,
    #[cfg(feature = "mobile-base")]
    pub wheels: WheelPins,
}

pub struct Leds {
    pub green: gpiob::PB0<Output<PushPull>>, // LD1
    pub blue: gpiob::PB7<Output<PushPull>>,  // LD3
    pub red: gpiob::PB14<Output<PushPull>>,  // LD2
}

pub struct Usart3Pins {
    pub tx: gpiod::PD8<Alternate<7>>,
    pub rx: gpiod::PD9<Alternate<7>>,
}

pub struct Spi1Pins {
    pub sck: gpioa::PA5<Alternate<5>>,
    pub miso: gpioa::PA6<Alternate<5>>,
    pub mosi: gpioa::PA7<Alternate<5>>,
    pub cs: gpioc::PC3<Output<PushPull>>,
    pub drdy: gpioa::PA3<Input<Floating>>,
}

pub struct Motor1Pins {
    pub in1: gpioc::PC6<Alternate<2>>, // TIM3_CH1 (PWM)
    pub in2: gpioc::PC7<Alternate<2>>, // TIM3_CH2 (PWM)
    pub cs: gpiod::PD0<Output<PushPull>>,
    pub nsleep: gpiod::PD1<Output<PushPull>>,
    pub disable: gpiod::PD2<Output<PushPull>>,
    pub adc: gpiob::PB1<Analog>,
}

pub struct I2c1Pins {
    pub scl: gpiob::PB8<Alternate<4, OpenDrain>>,
    pub sda: gpiob::PB9<Alternate<4, OpenDrain>>,
}

pub struct Motor2Pins {
    pub in1: gpioc::PC8<Alternate<2>>, // TIM3_CH3 (PWM)
    pub in2: gpioc::PC9<Alternate<2>>, // TIM3_CH4 (PWM)
    pub cs: gpiod::PD3<Output<PushPull>>,
    pub nsleep: gpiod::PD4<Output<PushPull>>,
    pub disable: gpiod::PD5<Output<PushPull>>,
    pub adc: gpioc::PC2<Analog>,
}

#[cfg(feature = "mobile-base")]
pub struct WheelPins {
    pub fl_pwm: gpiod::PD12<Alternate<2>>, // TIM4_CH1
    pub fl_in1: gpiod::PD6<Output<PushPull>>,
    pub fl_in2: gpiod::PD7<Output<PushPull>>,
    pub fr_pwm: gpiod::PD13<Alternate<2>>, // TIM4_CH2
    pub fr_in1: gpiod::PD11<Output<PushPull>>,
    pub fr_in2: gpioe::PE2<Output<PushPull>>,
    pub bl_pwm: gpiod::PD14<Alternate<2>>, // TIM4_CH3
    pub bl_in1: gpioa::PA1<Output<PushPull>>,
    pub bl_in2: gpioa::PA2<Output<PushPull>>,
    pub br_pwm: gpiod::PD15<Alternate<2>>, // TIM4_CH4
    pub br_in1: gpioa::PA4<Output<PushPull>>,
    pub br_in2: gpioa::PA8<Output<PushPull>>,
}

impl BoardPins {
    pub fn new(
        gpioa: pac::GPIOA,
        gpiob: pac::GPIOB,
        gpioc: pac::GPIOC,
        gpiod: pac::GPIOD,
        #[cfg(feature = "mobile-base")] gpioe: pac::GPIOE,
    ) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
        let gpioc = gpioc.split();
        let gpiod = gpiod.split();
        #[cfg(feature = "mobile-base")]
        let gpioe = gpioe.split();

        Self {
            leds: Leds {
                green: gpiob.pb0.into_push_pull_output(),
                blue: gpiob.pb7.into_push_pull_output(),
                red: gpiob.pb14.into_push_pull_output(),
            },

            usart3: Usart3Pins {
                tx: gpiod.pd8.into_alternate::<7>(),
                rx: gpiod.pd9.into_alternate::<7>(),
            },

            spi1: Spi1Pins {
                sck: gpioa.pa5.into_alternate::<5>(),
                miso: gpioa.pa6.into_alternate::<5>(),
                mosi: gpioa.pa7.into_alternate::<5>(),
                cs: gpioc.pc3.into_push_pull_output(),
                drdy: gpioa.pa3.into_floating_input(),
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

            #[cfg(feature = "mobile-base")]
            wheels: WheelPins {
                fl_pwm: gpiod.pd12.into_alternate::<2>(),
                fl_in1: gpiod.pd6.into_push_pull_output(),
                fl_in2: gpiod.pd7.into_push_pull_output(),
                fr_pwm: gpiod.pd13.into_alternate::<2>(),
                fr_in1: gpiod.pd11.into_push_pull_output(),
                fr_in2: gpioe.pe2.into_push_pull_output(),
                bl_pwm: gpiod.pd14.into_alternate::<2>(),
                bl_in1: gpioa.pa1.into_push_pull_output(),
                bl_in2: gpioa.pa2.into_push_pull_output(),
                br_pwm: gpiod.pd15.into_alternate::<2>(),
                br_in1: gpioa.pa4.into_push_pull_output(),
                br_in2: gpioa.pa8.into_push_pull_output(),
            },
        }
    }
}
