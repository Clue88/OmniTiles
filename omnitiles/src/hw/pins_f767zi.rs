// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F767ZI devboard.

use stm32f7xx_hal::{
    gpio::{gpioa, gpiob, gpioc, gpiod, Alternate, Analog, Floating, Input, Output, PushPull},
    pac,
    prelude::*,
};

pub struct BoardPins {
    pub leds: Leds,
    pub usart3: Usart3Pins,
    pub spi1: Spi1Pins,
    pub m1: Motor1Pins,
    pub m2: Motor2Pins,
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

pub struct Motor2Pins {
    pub in1: gpioc::PC8<Alternate<2>>, // TIM3_CH3 (PWM)
    pub in2: gpioc::PC9<Alternate<2>>, // TIM3_CH4 (PWM)
    pub cs: gpiod::PD3<Output<PushPull>>,
    pub nsleep: gpiod::PD4<Output<PushPull>>,
    pub disable: gpiod::PD5<Output<PushPull>>,
    pub adc: gpioc::PC2<Analog>,
}

impl BoardPins {
    pub fn new(gpioa: pac::GPIOA, gpiob: pac::GPIOB, gpioc: pac::GPIOC, gpiod: pac::GPIOD) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
        let gpioc = gpioc.split();
        let gpiod = gpiod.split();

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
        }
    }
}
