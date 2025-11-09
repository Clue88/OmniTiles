use embedded_hal::digital::OutputPin;

/// Whether the LED is driven active-high or active-low on the board wiring.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ActiveLevel {
    High,
    Low,
}

/// LED abstraction that remembers its active level and last known state.
pub struct Led<PIN: OutputPin> {
    pin: PIN,
    active: ActiveLevel,
    is_on: bool,
}

impl<PIN: OutputPin> Led<PIN> {
    /// Create an LED wrapper, initializing it to OFF.
    pub fn new(mut pin: PIN, active: ActiveLevel) -> Self {
        match active {
            ActiveLevel::High => pin.set_low().ok(),
            ActiveLevel::Low => pin.set_high().ok(),
        };
        Self {
            pin,
            active,
            is_on: false,
        }
    }

    /// Drive the LED logically ON (true) or OFF (false).
    pub fn set(&mut self, on: bool) {
        match (self.active, on) {
            (ActiveLevel::High, true) => self.pin.set_high().ok(),
            (ActiveLevel::High, false) => self.pin.set_low().ok(),
            (ActiveLevel::Low, true) => self.pin.set_low().ok(),
            (ActiveLevel::Low, false) => self.pin.set_high().ok(),
        };
        self.is_on = on;
    }

    #[inline]
    pub fn on(&mut self) {
        self.set(true);
    }

    #[inline]
    pub fn off(&mut self) {
        self.set(false);
    }

    pub fn toggle(&mut self) {
        self.set(!self.is_on);
    }

    #[inline]
    pub fn is_on(&self) -> bool {
        self.is_on
    }

    pub fn free(self) -> PIN {
        self.pin
    }
}

impl<PIN: OutputPin> Led<PIN> {
    pub fn active_high(pin: PIN) -> Self {
        Self::new(pin, ActiveLevel::High)
    }
    pub fn active_low(pin: PIN) -> Self {
        Self::new(pin, ActiveLevel::Low)
    }
}
