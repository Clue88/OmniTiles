//! LED abstraction layer.

use stm32f7xx_hal::gpio::{self, Output, PinState, PushPull};

/// Whether the LED is driven active-high or active-low on the board wiring.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ActiveLevel {
    High,
    Low,
}

/// LED abstraction that remembers its active level and last known state.
pub struct Led<const P: char, const N: u8> {
    pin: gpio::Pin<P, N, Output<PushPull>>,
    active: ActiveLevel,
}

impl<const P: char, const N: u8> Led<P, N> {
    /// Create an LED wrapper, initializing it to OFF.
    pub fn new<MODE>(pin: gpio::Pin<P, N, MODE>, active: ActiveLevel) -> Self {
        let mut pin = pin.into_push_pull_output();
        pin.set_state(match active {
            ActiveLevel::High => PinState::Low,
            ActiveLevel::Low => PinState::High,
        });
        Self { pin, active }
    }

    /// Drive the LED logically ON (true) or OFF (false).
    pub fn set(&mut self, on: bool) {
        match (self.active, on) {
            (ActiveLevel::High, true) => self.pin.set_high(),
            (ActiveLevel::High, false) => self.pin.set_low(),
            (ActiveLevel::Low, true) => self.pin.set_low(),
            (ActiveLevel::Low, false) => self.pin.set_high(),
        }
    }

    #[inline]
    pub fn on(&mut self) {
        self.set(true)
    }

    #[inline]
    pub fn off(&mut self) {
        self.set(false)
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        match self.active {
            ActiveLevel::High => self.pin.is_set_high(),
            ActiveLevel::Low => self.pin.is_set_low(),
        }
    }

    #[inline]
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    pub fn free(self) -> gpio::Pin<P, N, Output<PushPull>> {
        self.pin
    }
}

impl<const P: char, const N: u8> Led<P, N> {
    pub fn active_high<MODE>(pin: gpio::Pin<P, N, MODE>) -> Self {
        Self::new(pin, ActiveLevel::High)
    }
    pub fn active_low<MODE>(pin: gpio::Pin<P, N, MODE>) -> Self {
        Self::new(pin, ActiveLevel::Low)
    }
}
