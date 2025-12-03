#![no_main]
#![no_std]

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;

use hal::{
    can::Can,
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};
use stm32f7xx_hal as hal;

use omnitiles::{
    drivers::{Drv8873, Fit0185},
    hw::{adc::volts_from_adc, Adc, BoardPins, CanBus, ChipSelect, Encoder, Led, SpiBus, Usart},
};

#[entry]
fn main() -> ! {
    // ================================
    // Peripherals
    // ================================
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // ================================
    // Clocks
    // ================================
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let mut apb1 = rcc.apb1;
    let mut apb2 = rcc.apb2;

    // ================================
    // SysTick Delay
    // ================================
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // ================================
    // Board Pins
    // ================================
    let pins = BoardPins::new(dp.GPIOA, dp.GPIOB, dp.GPIOC, dp.GPIOD, dp.GPIOE, dp.GPIOH);

    // ================================
    // LEDs
    // ================================
    let mut led_yellow = Led::active_low(pins.leds.yellow);
    let mut led_green = Led::active_low(pins.leds.green);

    // ================================
    // USART1 Debug
    // ================================
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

    usart.println("Booting OmniTiles test firmware...");

    // ================================
    // SPI4 + Chip Selects
    // ================================
    let mut spi_bus = {
        let spi_mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnSecondTransition,
        };
        let spi4_raw = Spi::new(dp.SPI4, (pins.spi4.sck, pins.spi4.miso, pins.spi4.mosi));
        let spi4_enabled = spi4_raw.enable::<u8>(spi_mode, 10.kHz(), &clocks, &mut apb2);
        SpiBus::new(spi4_enabled)
    };
    let cs1 = ChipSelect::active_low(pins.drv8873.m1_cs);

    // ================================
    // CAN2 (Loopback)
    // ================================
    let pclk1_hz = clocks.pclk1().to_Hz();
    writeln!(usart, "PCLK1 = {} Hz\r", pclk1_hz).ok();
    const CAN_BTR: u32 = 0x001C_0003; // 250 kbps @ 16 MHz

    let mut can2_hal = Can::new(dp.CAN2, &mut apb1, (pins.can2.tx, pins.can2.rx));

    // For dual CAN filter setup, we need to configure filters on CAN1
    {
        let can1_hal = Can::new(dp.CAN1, &mut apb1, (pins.can1.tx, pins.can1.rx));
        let mut can1_bus = CanBus::new(can1_hal, CAN_BTR, false, false);
        can1_bus.configure_accept_all_filters_for_dual_can(&mut can2_hal);
    }

    let mut can_bus = CanBus::new(can2_hal, CAN_BTR, true, false); // loopback mode

    // ================================
    // TIM2 Encoder
    // ================================
    let enc = {
        let rcc_regs = unsafe { &*pac::RCC::ptr() };
        rcc_regs.apb1enr.modify(|_, w| w.tim2en().set_bit());
        Encoder::tim2(dp.TIM2)
    };

    // ================================
    // ADC1 Current Sense
    // ================================
    let adc1 = RefCell::new(Adc::adc1(dp.ADC1));

    let mut read_m1_iprop1 = Adc::make_reader(&adc1, 14);
    let mut read_m1_iprop2 = Adc::make_reader(&adc1, 15);

    // ================================
    // FIT0185 Motor
    // ================================
    let mut fit0185 = {
        let cpr = 2803;
        Fit0185::new(
            Drv8873::new(cs1),
            enc,
            pins.m1.in1,
            pins.m1.in2,
            pins.m1.nsleep,
            pins.m1.disable,
            cpr,
        )
    };
    fit0185.enable_outputs();

    // ================================
    // Main Program
    // ================================
    // ---- CAN Loopback Test ----
    usart.println("CAN loopback test (expected: [1, 2, 3, 4])...");
    let id = bxcan::StandardId::new(0x123).unwrap();
    let _ = can_bus.transmit_data(id, &[1, 2, 3, 4]);
    let frame = can_bus.receive().unwrap();
    match frame.data() {
        Some(data) => {
            let bytes: &[u8] = data.as_ref();
            writeln!(usart, "  CAN RX = {:?}\r", bytes).ok();
        }
        None => {
            writeln!(usart, "  CAN RX = <no data>\r").ok();
        }
    }

    // ---- FIT0185 Test ----
    usart.println("Reading FIT0185 status...");
    let spi_fault = fit0185.read_fault(&mut spi_bus);
    match spi_fault {
        Ok(fault) => {
            writeln!(usart, "  FIT0185 FAULT = {:?}\r", fault).ok();
        }
        Err(e) => {
            writeln!(usart, "  FIT0185 read_fault error: {:?}\r", e).ok();
        }
    }
    let spi_diag = fit0185.read_diag(&mut spi_bus);
    match spi_diag {
        Ok(diag) => {
            writeln!(usart, "  FIT0185 DIAG = {:?}\r", diag).ok();
        }
        Err(e) => {
            writeln!(usart, "  FIT0185 read_diag error: {:?}\r", e).ok();
        }
    }

    usart.println("Starting motor cycling test...");

    loop {
        usart.println("Motor FORWARD for 5 seconds...");
        fit0185.forward();
        for _ in 0..25 {
            led_green.toggle();

            let revs = fit0185.position_revs();
            let iprop1_amps = {
                let volts = volts_from_adc(read_m1_iprop1(), 3.3);
                (volts / 680.) * 1100.
            };
            let iprop2_amps = {
                let volts = volts_from_adc(read_m1_iprop2(), 3.3);
                (volts / 680.) * 1100.
            };
            let nfault = pins.m1.nfault.is_high();

            writeln!(
                usart,
                "Revs={}, IPROP1={} A, IPROP2={} A, nFAULT={}\r",
                revs, iprop1_amps, iprop2_amps, nfault
            )
            .ok();

            delay.delay_ms(200_u32);
        }

        usart.println("Motor STOP for 2 seconds...");
        fit0185.brake();
        led_green.off();
        delay.delay_ms(2000_u32);

        usart.println("Motor REVERSE for 5 seconds...");
        fit0185.reverse();
        for _ in 0..25 {
            led_yellow.toggle();

            let revs = fit0185.position_revs();
            let iprop1_amps = {
                let volts = volts_from_adc(read_m1_iprop1(), 3.3);
                (volts / 680.) * 1100.
            };
            let iprop2_amps = {
                let volts = volts_from_adc(read_m1_iprop2(), 3.3);
                (volts / 680.) * 1100.
            };
            let nfault = pins.m1.nfault.is_high();

            writeln!(
                usart,
                "Revs={}, IPROP1={} A, IPROP2={} A, nFAULT={}\r",
                revs, iprop1_amps, iprop2_amps, nfault
            )
            .ok();

            delay.delay_ms(200_u32);
        }

        usart.println("Motor STOP for 2 seconds...");
        fit0185.brake();
        led_yellow.off();
        delay.delay_ms(2000_u32);
    }
}
