#![no_main]
#![no_std]
#![allow(dead_code)]

use cortex_m_rt::entry;
use panic_halt as _;

use bxcan::StandardId;
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
use hw::{CanBus, ChipSelect, Encoder, Led, SpiBus, Usart};

mod drivers;
use drivers::Drv8873;

#[entry]
fn main() -> ! {
    // ========== Peripherals ==========
    let dp = pac::Peripherals::take().unwrap();

    // ========== Clocks ==========
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb1 = rcc.apb1;
    let mut apb2 = rcc.apb2;

    // ========== GPIO ==========
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();

    // ========== LED ==========
    let mut led_yellow = Led::active_low(gpiod.pd9);
    let mut led_green = Led::active_low(gpiod.pd10);

    // ========== USART1 (DBG) ==========
    let usart_tx = gpioa.pa9.into_alternate::<7>();
    let usart_rx = gpioa.pa10.into_alternate::<7>();
    let usart_cfg = Config {
        baud_rate: 115_200.bps(),
        ..Default::default()
    };
    let serial = Serial::new(dp.USART1, (usart_tx, usart_rx), &clocks, usart_cfg);
    let mut usart = Usart::new(serial);

    // ========== SPI4 ==========
    let sck = gpioe.pe12.into_alternate::<5>();
    let miso = gpioe.pe13.into_alternate::<5>();
    let mosi = gpioe.pe14.into_alternate::<5>();
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi4_raw = Spi::new(dp.SPI4, (sck, miso, mosi));
    let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
    let mut spi_bus = SpiBus::new(spi4_enabled);

    let cs1 = ChipSelect::active_low(gpioe.pe4);
    let cs2 = ChipSelect::active_low(gpioe.pe11);

    // ========== DRV8873 over SPI4 ==========
    let mut drv_m1 = Drv8873::new(cs1);
    let mut drv_m2 = Drv8873::new(cs2);

    // ========== CAN2 ==========
    let pclk1_hz = clocks.pclk1().to_Hz();
    writeln!(usart, "PCLK1 = {} Hz\r", pclk1_hz).ok(); // TODO: Use this to calculate CAN_BTR
    const CAN_BTR: u32 = 0x001c_0014; // TODO: Recalculate based on actual APB1 frequency

    let can1_tx = gpioa.pa12.into_alternate::<9>();
    let can1_rx = gpioa.pa11.into_alternate::<9>();
    let can1_hal = Can::new(dp.CAN1, &mut apb1, (can1_tx, can1_rx));
    let mut can1_bus = CanBus::new(can1_hal, CAN_BTR, false, false);
    can1_bus.configure_accept_all_filters(); // Configure filters on CAN1
    drop(can1_bus);

    let can2_tx = gpiob.pb13.into_alternate::<9>();
    let can2_rx = gpiob.pb12.into_alternate::<9>();
    let can2_hal = Can::new(dp.CAN2, &mut apb1, (can2_tx, can2_rx));
    let mut can_bus = CanBus::new(can2_hal, CAN_BTR, true, false); // Loopback mode

    // ========== TIM2/TIM3 ==========
    let _m1_enc_ch1 = gpioa.pa0.into_alternate::<1>();
    let _m1_enc_ch2 = gpioa.pa1.into_alternate::<1>();

    let _m2_enc_ch1 = gpioa.pa6.into_alternate::<2>();
    let _m2_enc_ch2 = gpioa.pa7.into_alternate::<2>();

    let rcc_regs = unsafe { &*pac::RCC::ptr() };
    rcc_regs.apb1enr.modify(|_, w| {
        w.tim2en().set_bit();
        w.tim3en().set_bit()
    });

    let tim2 = dp.TIM2;
    let tim3 = dp.TIM3;
    let mut enc1 = Encoder::tim2(tim2);
    let mut enc2 = Encoder::tim3(tim3);

    // ========== EXAMPLE USAGES ==========
    led_yellow.on();
    led_green.on();

    usart.println("Hello world!");

    if let Ok(resp) = drv_m1.read_fault(&mut spi_bus) {
        usart.print_hex_u8(resp.status.raw());
        usart.write_str(" FAULT=");
        usart.print_hex_u8(resp.data);
        usart.println("");
    }

    let _ = drv_m2.write_ic1(&mut spi_bus, 0x5A);

    let can_id = StandardId::new(0x123).unwrap();
    let _ = can_bus.transmit_data(can_id, &[0xDE, 0xAD, 0xBE, 0xEF]);

    enc1.reset();
    enc2.reset();

    loop {
        cortex_m::asm::nop();
    }
}
