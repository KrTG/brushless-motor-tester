use embedded_hal::digital::v2::InputPin;

pub struct Button<P> {
    pin: P,
    is_pressed: bool,
    start_time_ms: u32,
    last_impulse_duration_ms: u32,
    repeat_interval_ms: u32,
}

impl<P: InputPin> Button<P> {
    pub fn new(pin: P, repeat_interval_ms: u32) -> Self {
        Self {
            pin,
            is_pressed: false,
            start_time_ms: 0,
            last_impulse_duration_ms: 0,
            repeat_interval_ms,
        }
    }

    /// Update the button state and return 1 if an impulse occurred, 0 otherwise.
    pub fn update(&mut self, now_ms: u32) -> u32 {
        let mut impulse = 0;
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
                if duration >= 20 && self.last_impulse_duration_ms < 20 {
                    impulse = 1;
                    self.last_impulse_duration_ms = 20;
                }
                // 2. Long press (once at 300ms, then every repeat_interval_ms)
                else if duration >= 1000 {
                    if self.last_impulse_duration_ms < 1000 {
                        impulse = 1;
                        self.last_impulse_duration_ms = 1000;
                    } else if duration - self.last_impulse_duration_ms >= self.repeat_interval_ms {
                        impulse = 1;
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
        impulse
    }

    /// Returns true if the button is currently pressed.
    pub fn is_pressed(&self) -> bool {
        self.is_pressed
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
