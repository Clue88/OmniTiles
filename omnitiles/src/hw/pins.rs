//! Pin definitions for STM32F777 MCU for OmniTiles.

use stm32f7xx_hal::{
    gpio::{
        gpioa, gpiob, gpioc, gpiod, gpioe, gpioh, Alternate, Floating, Input, Output, PushPull,
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
    pub drv8873: DrvPins,
    pub m1: Motor1Pins,
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

/// SPI4 SCK/MISO/MOSI
pub struct Spi4Pins {
    pub sck: gpioe::PE12<Alternate<5>>,
    pub miso: gpioe::PE13<Alternate<5>>,
    pub mosi: gpioe::PE14<Alternate<5>>,
}

/// Chip-select and digital I/O for DRV8873 drivers
pub struct DrvPins {
    pub m1_cs: gpioe::PE4<Output<PushPull>>,
    pub m2_cs: gpioe::PE11<Output<PushPull>>,
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
    pub in1: gpioh::PH1<Output<PushPull>>,
    pub in2: gpioc::PC0<Output<PushPull>>,
    pub nsleep: gpioa::PA4<Input<Floating>>,
    pub disable: gpioa::PA3<Output<PushPull>>,
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
            },

            drv8873: DrvPins {
                m1_cs: gpioe.pe4.into_push_pull_output(),
                m2_cs: gpioe.pe11.into_push_pull_output(),
            },

            encoder: EncoderPins {
                tim2_ch1: gpioa.pa0.into_alternate::<1>(),
                tim2_ch2: gpioa.pa1.into_alternate::<1>(),
                tim3_ch1: gpioa.pa6.into_alternate::<2>(),
                tim3_ch2: gpioa.pa7.into_alternate::<2>(),
            },

            m1: Motor1Pins {
                in1: gpioh.ph1.into_push_pull_output(),
                in2: gpioc.pc0.into_push_pull_output(),
                nsleep: gpioa.pa4.into_floating_input(),
                disable: gpioa.pa3.into_push_pull_output(),
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
