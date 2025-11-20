//! Central definition of all STM32F777 pin assignments used by OmniTiles.

use stm32f7xx_hal::{gpio, pac, prelude::*};

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
    pub encoder: EncoderPins,
    pub can1: Can1Pins,
    pub can2: Can2Pins,
}

/// LED outputs
pub struct LedPins {
    pub red: gpio::Pin<'D', 8, gpio::Output<gpio::PushPull>>,
    pub yellow: gpio::Pin<'D', 9, gpio::Output<gpio::PushPull>>,
    pub green: gpio::Pin<'D', 10, gpio::Output<gpio::PushPull>>,
}

/// USART1 TX/RX
pub struct Usart1Pins {
    pub tx: gpio::Pin<'A', 9, gpio::Alternate<7>>,
    pub rx: gpio::Pin<'A', 10, gpio::Alternate<7>>,
}

/// SPI4 SCK/MISO/MOSI
pub struct Spi4Pins {
    pub sck: gpio::Pin<'E', 12, gpio::Alternate<5>>,
    pub miso: gpio::Pin<'E', 13, gpio::Alternate<5>>,
    pub mosi: gpio::Pin<'E', 14, gpio::Alternate<5>>,
}

/// Chip-select and digital I/O for DRV8873 drivers
pub struct DrvPins {
    pub m1_cs: gpio::Pin<'E', 4, gpio::Output<gpio::PushPull>>,
    pub m2_cs: gpio::Pin<'E', 11, gpio::Output<gpio::PushPull>>,
}

/// TIM2/TIM3 Quadrature Encoder Inputs
pub struct EncoderPins {
    pub tim2_ch1: gpio::Pin<'A', 0, gpio::Alternate<1>>,
    pub tim2_ch2: gpio::Pin<'A', 1, gpio::Alternate<1>>,

    pub tim3_ch1: gpio::Pin<'A', 6, gpio::Alternate<2>>,
    pub tim3_ch2: gpio::Pin<'A', 7, gpio::Alternate<2>>,
}

/// CAN1 bus pins
pub struct Can1Pins {
    pub tx: gpio::Pin<'A', 12, gpio::Alternate<9>>,
    pub rx: gpio::Pin<'A', 11, gpio::Alternate<9>>,
}

/// CAN2 bus pins
pub struct Can2Pins {
    pub tx: gpio::Pin<'B', 13, gpio::Alternate<9>>,
    pub rx: gpio::Pin<'B', 12, gpio::Alternate<9>>,
}

impl BoardPins {
    /// Create all named pins from raw GPIO peripherals.
    pub fn new(gpioa: pac::GPIOA, gpiob: pac::GPIOB, gpiod: pac::GPIOD, gpioe: pac::GPIOE) -> Self {
        let gpioa = gpioa.split();
        let gpiob = gpiob.split();
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

            can1: Can1Pins {
                tx: gpioa.pa12.into_alternate::<9>(),
                rx: gpioa.pa11.into_alternate::<9>(),
            },

            can2: Can2Pins {
                tx: gpiob.pb13.into_alternate::<9>(),
                rx: gpiob.pb12.into_alternate::<9>(),
            },
        }
    }
}
