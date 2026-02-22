use embedded_hal::digital::v2::InputPin;

pub struct Button<P> {
    pin: P,
    is_pressed: bool,
    press_duration: u32,
    last_impulse_time: u32,
    repeat_interval_ms: u32,
}

impl<P: InputPin> Button<P> {
    pub fn new(pin: P, repeat_interval_ms: u32) -> Self {
        Self {
            pin,
            is_pressed: false,
            press_duration: 0,
            last_impulse_time: 0,
            repeat_interval_ms,
        }
    }

    /// Update the button state and return 1 if an impulse occurred, 0 otherwise.
    /// Should be called every 1ms.
    pub fn update(&mut self) -> u32 {
        let mut impulse = 0;
        // Pins are usually pulled up, so low means pressed.
        let is_low = self.pin.is_low().ok().unwrap_or(false);

        if is_low {
            if !self.is_pressed {
                // Initial press detect
                self.is_pressed = true;
                self.press_duration = 0;
                self.last_impulse_time = 0;
            } else {
                // Button is being held
                self.press_duration += 1;

                // Impulse logic:
                // 1. Short press
                if self.press_duration == 40 {
                    impulse = 1;
                    self.last_impulse_time = 40;
                }
                // 2. Long press
                else if self.press_duration >= 300 {
                    if self.press_duration == 300 {
                        impulse = 1;
                        self.last_impulse_time = 300;
                    } else if self.press_duration - self.last_impulse_time
                        >= self.repeat_interval_ms
                    {
                        impulse = 1;
                        self.last_impulse_time = self.press_duration;
                    }
                }
            }
        } else {
            // Button is released
            self.is_pressed = false;
            self.press_duration = 0;
            self.last_impulse_time = 0;
        }
        impulse
    }

    /// Returns true if the button is currently pressed.
    pub fn is_pressed(&self) -> bool {
        self.is_pressed
    }

    /// Returns the duration the button has been pressed in milliseconds.
    pub fn press_duration(&self) -> u32 {
        if self.is_pressed {
            self.press_duration
        } else {
            0
        }
    }
}
