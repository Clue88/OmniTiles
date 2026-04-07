// SPDX-License-Identifier: MIT
// © 2025–2026 Christopher Liu

//! VL53L0X Time-of-Flight sensor driver.

use crate::hw::I2cBus;
use cortex_m::asm;
use stm32f7xx_hal::i2c::{self, PinScl, PinSda};
use stm32f7xx_hal::pac::I2C1;

const DEFAULT_ADDR: u8 = 0x29;

#[derive(Debug)]
pub enum Error {
    I2c(i2c::Error),
    InvalidDevice,
    Timeout,
}

impl From<i2c::Error> for Error {
    fn from(e: i2c::Error) -> Self {
        Error::I2c(e)
    }
}

pub struct Vl53l0x<SCL, SDA> {
    bus: I2cBus<SCL, SDA>,
    addr: u8,
    stop_variable: u8,
}

impl<SCL, SDA> Vl53l0x<SCL, SDA>
where
    SCL: PinScl<I2C1>,
    SDA: PinSda<I2C1>,
{
    /// Verify device identity and run mandatory data init (2V8 I/O, stop_variable, MSRC, sequence).
    pub fn new(bus: I2cBus<SCL, SDA>) -> Result<Self, Error> {
        let mut dev = Self {
            bus,
            addr: DEFAULT_ADDR,
            stop_variable: 0,
        };

        // Soft reset via SOFT_RESET_GO2_SOFT_RESET_N (0xBF): pull low then high.
        // Page-select (0xFF) and power (0x80) writes ensure the device is in a known
        // state. Errors are ignored because the device may NACK if stuck.
        let _ = dev.write_reg(0xFF, 0x00);
        let _ = dev.write_reg(0x80, 0x00);
        let _ = dev.write_reg(0xBF, 0x00);
        asm::delay(1_000_000);
        let _ = dev.write_reg(0xBF, 0x01);
        asm::delay(1_000_000);

        // IDENTIFICATION_MODEL_ID — must read 0xEE for VL53L0X
        if dev.read_reg(0xC0)? != 0xEE {
            return Err(Error::InvalidDevice);
        }

        // VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV — set bit 0 to enable 2.8V I/O mode
        let vhv = dev.read_reg(0x89)?;
        dev.write_reg(0x89, vhv | 0x01)?;

        // Capture stop_variable from undocumented register 0x91.
        // This device-specific value is needed later to start single-shot measurements.
        dev.write_reg(0x88, 0x00)?;
        dev.write_reg(0x80, 0x01)?;
        dev.write_reg(0xFF, 0x01)?;
        dev.write_reg(0x00, 0x00)?;
        dev.stop_variable = dev.read_reg(0x91)?;
        dev.write_reg(0x00, 0x01)?;
        dev.write_reg(0xFF, 0x00)?;
        dev.write_reg(0x80, 0x00)?;

        // MSRC_CONFIG_CONTROL — disable SIGNAL_RATE_MSRC (bit 1) and
        // SIGNAL_RATE_PRE_RANGE (bit 4) limit checks to avoid rejecting valid readings
        let msrc = dev.read_reg(0x60)?;
        dev.write_reg(0x60, msrc | 0x12)?;

        // FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT — 0.25 MCPS (9.7 fixed-point)
        dev.write_reg(0x44, 0x00)?;
        dev.write_reg(0x45, 0x20)?;

        dev.write_reg(0x01, 0xFF)?;

        Ok(dev)
    }

    /// Read SPAD info from NVM and configure the SPAD enable map.
    pub fn static_init(&mut self) -> Result<(), Error> {
        // Enter NVM read mode via private registers
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        self.write_reg(0xFF, 0x06)?;
        let r83 = self.read_reg(0x83)?;
        self.write_reg(0x83, r83 | 0x04)?;
        self.write_reg(0xFF, 0x07)?;
        self.write_reg(0x81, 0x01)?;
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0x94, 0x6B)?;
        self.write_reg(0x83, 0x00)?;

        // Wait for NVM read complete
        let mut ready = false;
        for _ in 0..50_000u32 {
            if self.read_reg(0x83)? != 0x00 {
                ready = true;
                break;
            }
        }
        if !ready {
            return Err(Error::Timeout);
        }

        self.write_reg(0x83, 0x01)?;
        let tmp = self.read_reg(0x92)?;
        let spad_count = tmp & 0x7F;
        let spad_is_aperture = (tmp >> 7) & 0x01 != 0;

        // Exit NVM read mode
        self.write_reg(0x81, 0x00)?;
        self.write_reg(0xFF, 0x06)?;
        let r83 = self.read_reg(0x83)?;
        self.write_reg(0x83, r83 & !0x04)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x4F, 0x00)?;
        self.write_reg(0x4E, 0x2C)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0xB6, 0xB4)?;

        // Read SPAD enable map (6 bytes from 0xB0)
        let mut spad_map = [0u8; 6];
        self.bus.write_read(self.addr, &[0xB0u8], &mut spad_map)?;

        // Enable first spad_count SPADs (aperture from bit 12, else from 0)
        let first_spad_to_enable: u8 = if spad_is_aperture { 12 } else { 0 };
        let mut spads_enabled: u8 = 0;

        for i in 0u8..48 {
            let byte = (i / 8) as usize;
            let bit = i % 8;
            if i < first_spad_to_enable || spads_enabled >= spad_count {
                spad_map[byte] &= !(1 << bit);
            } else if spad_map[byte] & (1 << bit) != 0 {
                spads_enabled += 1;
            }
        }

        // Write back SPAD map (7-byte I2C transaction)
        let mut buf = [0u8; 7];
        buf[0] = 0xB0;
        buf[1..7].copy_from_slice(&spad_map);
        self.bus.write(self.addr, &buf)?;

        Ok(())
    }

    /// Write ST's default tuning register table.
    ///
    /// This is a magic blob copied verbatim from ST's proprietary VL53L0X API
    /// (vl53l0x_tuning.h). It configures internal analog/digital parameters that
    /// ST does not publicly document. Every open-source VL53L0X driver reproduces
    /// this table as-is. Do not modify individual values.
    pub fn load_tuning(&mut self) -> Result<(), Error> {
        let settings: &[(u8, u8)] = &[
            (0xFF, 0x01),
            (0x00, 0x00),
            (0xFF, 0x00),
            (0x09, 0x00),
            (0x10, 0x00),
            (0x11, 0x00),
            (0x24, 0x01),
            (0x25, 0xFF),
            (0x75, 0x00),
            (0xFF, 0x01),
            (0x4E, 0x2C),
            (0x48, 0x00),
            (0x30, 0x20),
            (0xFF, 0x00),
            (0x30, 0x09),
            (0x54, 0x00),
            (0x31, 0x04),
            (0x32, 0x03),
            (0x40, 0x83),
            (0x46, 0x25),
            (0x60, 0x00),
            (0x27, 0x00),
            (0x50, 0x06),
            (0x51, 0x00),
            (0x52, 0x96),
            (0x56, 0x08),
            (0x57, 0x30),
            (0x61, 0x00),
            (0x62, 0x00),
            (0x64, 0x00),
            (0x65, 0x00),
            (0x66, 0xA0),
            (0xFF, 0x01),
            (0x22, 0x32),
            (0x47, 0x14),
            (0x49, 0xFF),
            (0x4A, 0x00),
            (0xFF, 0x00),
            (0x7A, 0x0A),
            (0x7B, 0x00),
            (0x78, 0x21),
            (0xFF, 0x01),
            (0x23, 0x34),
            (0x42, 0x00),
            (0x44, 0xFF),
            (0x45, 0x26),
            (0x46, 0x05),
            (0x40, 0x40),
            (0x0E, 0x06),
            (0x20, 0x1A),
            (0x43, 0x40),
            (0xFF, 0x00),
            (0x34, 0x03),
            (0x35, 0x44),
            (0xFF, 0x01),
            (0x31, 0x04),
            (0x4B, 0x09),
            (0x4C, 0x05),
            (0x4D, 0x04),
            (0xFF, 0x00),
            (0x44, 0x00),
            (0x45, 0x20),
            (0x47, 0x08),
            (0x48, 0x28),
            (0x67, 0x00),
            (0x70, 0x04),
            (0x71, 0x01),
            (0x72, 0xFE),
            (0x76, 0x00),
            (0x77, 0x00),
            (0xFF, 0x01),
            (0x0D, 0x01),
            (0xFF, 0x00),
            (0x80, 0x01),
            (0x01, 0xF8),
            (0xFF, 0x01),
            (0x8E, 0x01),
            (0x00, 0x01),
            (0xFF, 0x00),
            (0x80, 0x00),
        ];
        for &(reg, val) in settings {
            self.write_reg(reg, val)?;
        }
        Ok(())
    }

    /// Run VHV (voltage) and phase reference calibration.
    ///
    /// Must be called after `load_tuning`. The two calibration passes (VHV = 0x40,
    /// phase = 0x00) compensate for chip-to-chip variation. Register 0x01 is
    /// SYSTEM_SEQUENCE_CONFIG — each write selects which calibration step runs.
    pub fn calibrate(&mut self) -> Result<(), Error> {
        // SYSRANGE_START config and clear GPIO interrupt
        self.write_reg(0x0A, 0x04)?;
        let gpio_hv = self.read_reg(0x84)?;
        self.write_reg(0x84, gpio_hv & !0x10)?;
        self.write_reg(0x0B, 0x01)?;

        self.write_reg(0x01, 0xFF)?;

        // VHV calibration
        self.write_reg(0x01, 0x01)?;
        self.perform_single_ref_calibration(0x40)?;

        // Phase calibration
        self.write_reg(0x01, 0x02)?;
        self.perform_single_ref_calibration(0x00)?;

        // Restore full sequence config
        self.write_reg(0x01, 0xFF)?;

        Ok(())
    }

    /// Single-shot range measurement in mm
    pub fn read_range_mm(&mut self) -> Result<u16, Error> {
        // Write stop_variable (captured during init) then trigger single-shot via
        // SYSRANGE_START (0x00)
        self.write_reg(0x80, 0x01)?;
        self.write_reg(0xFF, 0x01)?;
        self.write_reg(0x00, 0x00)?;
        self.write_reg(0x91, self.stop_variable)?;
        self.write_reg(0x00, 0x01)?;
        self.write_reg(0xFF, 0x00)?;
        self.write_reg(0x80, 0x00)?;

        self.write_reg(0x00, 0x01)?;

        // Wait for SYSRANGE_START bit 0 to clear (device accepted the command)
        let mut started = false;
        for _ in 0..50_000u32 {
            if self.read_reg(0x00)? & 0x01 == 0 {
                started = true;
                break;
            }
        }
        if !started {
            return Err(Error::Timeout);
        }

        // Wait for RESULT_INTERRUPT_STATUS (0x13) — bits [2:0] != 0 means data ready
        let mut done = false;
        for _ in 0..50_000u32 {
            if self.read_reg(0x13)? & 0x07 != 0 {
                done = true;
                break;
            }
        }
        if !done {
            return Err(Error::Timeout);
        }

        // Read 16-bit range from RESULT_RANGE_STATUS + 10 (0x1E..0x1F)
        let mut buf = [0u8; 2];
        self.bus.write_read(self.addr, &[0x1Eu8], &mut buf)?;
        let mm = ((buf[0] as u16) << 8) | buf[1] as u16;

        // SYSTEM_INTERRUPT_CLEAR — acknowledge the interrupt
        self.write_reg(0x0B, 0x01)?;

        Ok(mm)
    }

    /// Start a calibration via SYSRANGE_START, poll RESULT_INTERRUPT_STATUS until
    /// complete, then clear the interrupt. `vhv_init_byte` selects the cal type:
    /// 0x40 = VHV, 0x00 = phase.
    fn perform_single_ref_calibration(&mut self, vhv_init_byte: u8) -> Result<(), Error> {
        self.write_reg(0x00, 0x01 | vhv_init_byte)?;

        let mut done = false;
        for _ in 0..50_000u32 {
            if self.read_reg(0x13)? & 0x07 != 0 {
                done = true;
                break;
            }
        }
        if !done {
            return Err(Error::Timeout);
        }

        self.write_reg(0x0B, 0x01)?;
        self.write_reg(0x00, 0x00)?;

        Ok(())
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
        self.bus.write(self.addr, &[reg, val]).map_err(Error::I2c)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.bus.write_read(self.addr, &[reg], &mut buf)?;
        Ok(buf[0])
    }
}
