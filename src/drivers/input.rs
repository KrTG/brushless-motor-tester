use embedded_hal::digital::v2::InputPin;

pub struct Button<P> {
    pin: P,
    is_pressed: bool,
    start_time_ms: u32,
    last_impulse_duration_ms: u32,
    repeat_interval_ms: u32,
    pulses: u32,
}

impl<P: InputPin> Button<P> {
    pub fn new(pin: P, repeat_interval_ms: u32) -> Self {
        Self {
            pin,
            is_pressed: false,
            start_time_ms: 0,
            last_impulse_duration_ms: 0,
            repeat_interval_ms,
            pulses: 0,
        }
    }

    /// Update the button state and internal pulses.
    pub fn update(&mut self, now_ms: u32) {
        // Pins are usually pulled up, so low means pressed.
        let is_low = self.pin.is_low().ok().unwrap_or(false);

        if is_low {
            if !self.is_pressed {
                // Initial press detect
                self.is_pressed = true;
                self.start_time_ms = now_ms;
                self.last_impulse_duration_ms = 0;
            } else {
                // Button is being held
                let duration = now_ms.wrapping_sub(self.start_time_ms);

                // Impulse logic:
                // 1. Short press (once at 40ms)
                if duration >= 40 && self.last_impulse_duration_ms < 40 {
                    self.pulses += 1;
                    self.last_impulse_duration_ms = 40;
                }
                // 2. Long press (once at 300ms, then every repeat_interval_ms)
                else if duration >= 1000 {
                    if self.last_impulse_duration_ms < 1000 {
                        self.pulses += 1;
                        self.last_impulse_duration_ms = 1000;
                    } else if duration - self.last_impulse_duration_ms >= self.repeat_interval_ms {
                        self.pulses += 1;
                        self.last_impulse_duration_ms += self.repeat_interval_ms;
                    }
                }
            }
        } else {
            // Button is released
            self.is_pressed = false;
            self.start_time_ms = 0;
            self.last_impulse_duration_ms = 0;
        }
    }

    /// Returns the number of pulses since the last call and resets the counter.
    pub fn get_and_reset_pulses(&mut self) -> u32 {
        let p = self.pulses;
        self.pulses = 0;
        p
    }

    /// Returns true if the button is currently pressed.
    pub fn probe(&self) -> bool {
        self.pin.is_low().ok().unwrap_or(false)
    }

    /// Returns the duration the button has been pressed in milliseconds.
    pub fn press_duration(&self, now_ms: u32) -> u32 {
        if self.is_pressed {
            now_ms.wrapping_sub(self.start_time_ms)
        } else {
            0
        }
    }
}
