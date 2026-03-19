use embedded_hal::digital::v2::InputPin;

pub struct Button<P> {
    pin: P,
    is_pressed: bool,
    last_raw_state: bool,
    last_bounce_time_ms: u32,
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
            last_raw_state: false,
            last_bounce_time_ms: 0,
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
        const DEBOUNCE_DELAY_MS: u32 = 20;

        // If the physical pin state changes, mark the time of the bounce
        if is_low != self.last_raw_state {
            self.last_bounce_time_ms = now_ms;
            self.last_raw_state = is_low;
        }

        // If the physical state has been stable longer than the debounce delay
        if now_ms.wrapping_sub(self.last_bounce_time_ms) >= DEBOUNCE_DELAY_MS {
            // And if this debounced state is different from our current logical state
            if is_low != self.is_pressed {
                self.is_pressed = is_low;

                if self.is_pressed {
                    // Button has been cleanly pressed
                    self.pulses += 1;
                    self.start_time_ms = now_ms;
                    self.last_impulse_duration_ms = 0;
                } else {
                    // Button has been cleanly released
                    self.start_time_ms = 0;
                    self.last_impulse_duration_ms = 0;
                }
            }
        }

        // If the button is currently being held down securely
        if self.is_pressed {
            let duration = now_ms.wrapping_sub(self.start_time_ms);

            // Handle press-and-hold repeating (once at 1000ms, then every repeat_interval_ms)
            if duration >= 1000 {
                if self.last_impulse_duration_ms < 1000 {
                    self.pulses += 1;
                    self.last_impulse_duration_ms = 1000;
                } else if duration.wrapping_sub(self.last_impulse_duration_ms)
                    >= self.repeat_interval_ms
                {
                    self.pulses += 1;
                    self.last_impulse_duration_ms += self.repeat_interval_ms;
                }
            }
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
