#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use core::fmt::Write;

use hal::{
    can::Can,
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

mod hw;
use hw::{BoardPins, CanBus, ChipSelect, Encoder, Led, SpiBus, Usart};

mod drivers;

#[entry]
fn main() -> ! {
    // ========== Peripherals ==========
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // ========== Clocks ==========
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb1 = rcc.apb1;
    let mut apb2 = rcc.apb2;

    // ========== SysTick Delay ==========
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // ========== Pins ==========
    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOD, dp.GPIOE);

    // ========== LED ==========
    let mut led_yellow = Led::active_low(pins.leds.yellow);
    let mut led_green = Led::active_low(pins.leds.green);

    // ========== USART1 (DBG) ==========
    let serial = Serial::new(
        dp.USART1,
        (pins.usart1.tx, pins.usart1.rx),
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            ..Default::default()
        },
    );
    let mut usart = Usart::new(serial);

    // ========== SPI4 ==========
    let mut spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let spi4_raw = Spi::new(dp.SPI4, (pins.spi4.sck, pins.spi4.miso, pins.spi4.mosi));
        let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi4_enabled)
    };
    let mut cs1 = ChipSelect::active_low(pins.drv8873.m1_cs);

    // ========== CAN2 Loopback ==========
    let pclk1_hz = clocks.pclk1().to_Hz();
    writeln!(usart, "PCLK1 = {} Hz\r", pclk1_hz).ok(); // TODO: Use this to calculate CAN_BTR
    const CAN_BTR: u32 = 0x001c_0014; // TODO: Recalculate based on actual APB1 frequency

    let mut can1_bus = {
        let can1_hal = Can::new(dp.CAN1, &mut apb1, (pins.can1.tx, pins.can1.rx));
        CanBus::new(can1_hal, CAN_BTR, false, false)
    };
    can1_bus.configure_accept_all_filters(); // Configure filters on CAN1
    drop(can1_bus);

    let mut can_bus = {
        let can2_hal = Can::new(dp.CAN2, &mut apb1, (pins.can2.tx, pins.can2.rx));
        CanBus::new(can2_hal, CAN_BTR, true, false) // Loopback mode
    };

    // ========== TIM2 Encoder ==========
    let enc = {
        let rcc_regs = unsafe { &*pac::RCC::ptr() };
        rcc_regs.apb1enr.modify(|_, w| w.tim2en().set_bit());
        Encoder::tim2(dp.TIM2)
    };

    // ========== HARDWARE TEST ==========
    led_yellow.on();

    usart.println("Hello world!");

    writeln!(usart, "SPI test...").ok();
    cs1.select();
    let r = spi_bus.transfer_byte(0xAA);
    cs1.deselect();
    writeln!(usart, "SPI RX={:?}", r).ok();

    writeln!(usart, "CAN loopback test...").ok();
    let id = bxcan::StandardId::new(0x123).unwrap();
    let _ = can_bus.transmit_data(id, &[1, 2, 3, 4]);
    let frame = can_bus.receive().unwrap();
    writeln!(usart, "CAN RX={:?}", frame.data()).ok();

    writeln!(usart, "Spin encoder to see values").ok();

    loop {
        led_green.toggle();
        writeln!(usart, "ENC={}", enc.position()).ok();
        delay.delay_ms(20_u32);
    }
}
