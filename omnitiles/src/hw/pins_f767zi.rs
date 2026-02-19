// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F767ZI devboard.

use stm32f7xx_hal::{
    gpio::{gpioa, gpiob, gpioc, gpiod, Alternate, Floating, Input, Output, PushPull},
    pac,
    prelude::*,
};

pub struct BoardPins {
    pub leds: Leds,
    pub usart3: Usart3Pins,
    pub spi1: Spi1Pins,
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

pub struct Can1Pins {
    pub tx: gpioa::PA12<Alternate<9>>,
    pub rx: gpioa::PA11<Alternate<9>>,
}

pub struct EncoderPins {
    pub ch1: gpioa::PA0<Alternate<1>>,
    pub ch2: gpioa::PA1<Alternate<1>>,
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
        }
    }
}
