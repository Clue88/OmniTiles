//! Pin definitions for the NUCLEO-F767ZI development board.
//!
//! This allows for some amount of firmware testing if only the development board is available.

use stm32f7xx_hal::{
    gpio::{gpioa, gpiob, gpiod, gpioe, Alternate, Output, PushPull},
    pac,
    prelude::*,
};

pub struct BoardPins {
    pub leds: Leds,
    pub usart3: Usart3Pins,
    pub spi4: Spi4Pins,
    pub can1: Can1Pins,
    pub encoder: EncoderPins,
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

pub struct Spi4Pins {
    pub sck: gpioe::PE2<Alternate<5>>,
    pub miso: gpioe::PE5<Alternate<5>>,
    pub mosi: gpioe::PE6<Alternate<5>>,
    pub cs: gpioe::PE4<Output<PushPull>>,
}

pub struct Can1Pins {
    pub tx: gpiod::PD1<Alternate<9>>,
    pub rx: gpiod::PD0<Alternate<9>>,
}

pub struct EncoderPins {
    pub ch1: gpioa::PA0<Alternate<1>>,
    pub ch2: gpioa::PA1<Alternate<1>>,
}

impl BoardPins {
    pub fn new(gpioa: pac::GPIOA, gpiob: pac::GPIOB, gpiod: pac::GPIOD, gpioe: pac::GPIOE) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
        let gpiod = gpiod.split();
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

            spi4: Spi4Pins {
                sck: gpioe.pe2.into_alternate::<5>(),
                miso: gpioe.pe5.into_alternate::<5>(),
                mosi: gpioe.pe6.into_alternate::<5>(),
                cs: gpioe.pe4.into_push_pull_output(),
            },

            can1: Can1Pins {
                tx: gpiod.pd1.into_alternate::<9>(),
                rx: gpiod.pd0.into_alternate::<9>(),
            },

            encoder: EncoderPins {
                ch1: gpioa.pa0.into_alternate::<1>(),
                ch2: gpioa.pa1.into_alternate::<1>(),
            },
        }
    }
}
