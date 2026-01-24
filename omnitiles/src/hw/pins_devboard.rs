// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! Pin definitions for STM32F777 MCU Devboard for OmniTiles.

use stm32f7xx_hal::{
    gpio::{gpioa, gpiod, gpioe, Alternate, Output, PushPull},
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
    pub cs1: gpioe::PE4<Output<PushPull>>,
    pub cs2: gpioe::PE11<Output<PushPull>>,
}

/// CAN1 TX/RX
pub struct Can1Pins {
    pub tx: gpioa::PA12<Alternate<9>>,
    pub rx: gpioa::PA11<Alternate<9>>,
}

impl BoardPins {
    /// Create all named pins from raw GPIO peripherals.
    pub fn new(gpioa: pac::GPIOA, gpiod: pac::GPIOD, gpioe: pac::GPIOE) -> Self {
        let gpioa = gpioa.split();
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
                cs1: gpioe.pe4.into_push_pull_output(),
                cs2: gpioe.pe11.into_push_pull_output(),
            },

            can1: Can1Pins {
                tx: gpioa.pa12.into_alternate::<9>(),
                rx: gpioa.pa11.into_alternate::<9>().internal_pull_up(true),
            },
        }
    }
}
