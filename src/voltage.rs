use embedded_hal::adc::OneShot;
use stm32f7xx_hal::{adc::Adc, pac::ADC1};

pub struct VoltageSensor<PIN, const N: usize = 30> {
    adc: Adc<ADC1>,
    pin: PIN,
    voltage_divider: f32,
    buffer: [f32; N],
    index: usize,
    sum: f32,
    initialized: bool,
    last_update_ms: u32,
    sample_interval_ms: u32,
}

impl<PIN, const N: usize> VoltageSensor<PIN, N>
where
    Adc<ADC1>: OneShot<ADC1, u16, PIN>,
    PIN: embedded_hal::adc::Channel<ADC1>,
{
    pub fn new(adc: Adc<ADC1>, pin: PIN, voltage_divider: f32, window_duration_ms: u32) -> Self {
        Self {
            adc,
            pin,
            voltage_divider,
            buffer: [0.0; N],
            index: 0,
            sum: 0.0,
            initialized: false,
            last_update_ms: 0,
            sample_interval_ms: window_duration_ms / (N as u32),
        }
    }

    pub fn read(&mut self, now_ms: u32) -> f32 {
        let sample = loop {
            match self.adc.read(&mut self.pin) {
                Ok(sample) => break sample,
                Err(_) => continue,
            }
        };
        let val = (sample as f32) * (3.3 / 4095.0) * self.voltage_divider;

        if !self.initialized {
            for i in 0..N {
                self.buffer[i] = val;
            }
            self.sum = val * (N as f32);
            self.initialized = true;
            self.last_update_ms = now_ms;
            return val;
        }

        // Only update the rolling window if the interval has passed
        if now_ms.wrapping_sub(self.last_update_ms) >= self.sample_interval_ms {
            // Subtract the old value and add the new one
            self.sum -= self.buffer[self.index];
            self.buffer[self.index] = val;
            self.sum += val;

            // Move the index
            self.index = (self.index + 1) % N;
            self.last_update_ms = now_ms;
        }

        // Return the current average
        self.sum / (N as f32)
    }
}
