use embedded_hal::adc::OneShot;
use stm32f7xx_hal::adc::Adc;

pub struct CurrentSensor<ADC, PIN, const N: usize = 30> {
    adc: Adc<ADC>,
    pin: PIN,
    voltage_divider: f32,
    sensitivity: f32, // V/A
    buffer: [f32; N],
    index: usize,
    sum: f32,
    initialized: bool,
    last_update_ms: u32,
    sample_interval_ms: u32,
    vdda: f32,
    zero_offset: f32,
}

impl<ADC, PIN, const N: usize> CurrentSensor<ADC, PIN, N>
where
    Adc<ADC>: OneShot<ADC, u16, PIN>,
    PIN: embedded_hal::adc::Channel<ADC>,
{
    pub fn new(
        adc: Adc<ADC>,
        pin: PIN,
        voltage_divider: f32,
        sensitivity_mv_a: f32,
        sample_interval_ms: u32,
        vdda: f32,
    ) -> Self {
        let mut instance = Self {
            adc,
            pin,
            voltage_divider,
            sensitivity: sensitivity_mv_a / 1000.0,
            buffer: [0.0; N],
            index: 0,
            sum: 0.0,
            initialized: false,
            last_update_ms: 0,
            sample_interval_ms,
            vdda,
            zero_offset: 2.5,
        };
        instance.measure();
        instance
    }

    pub fn sample(&mut self, now_ms: u32) {
        if self.initialized && now_ms - self.last_update_ms < self.sample_interval_ms {
            return;
        }
        self.measure();
        self.last_update_ms = now_ms;
    }

    fn measure(&mut self) {
        let sample = loop {
            match self.adc.read(&mut self.pin) {
                Ok(sample) => break sample,
                Err(_) => continue,
            }
        };

        // Calculate voltage at the sensor's output pin
        let v_adc = (sample as f32) * (self.vdda / 4095.0);
        let v_out = v_adc * self.voltage_divider;

        if !self.initialized {
            for i in 0..N {
                self.buffer[i] = v_out;
            }
            self.sum = v_out * (N as f32);
            self.initialized = true;
        } else {
            self.sum -= self.buffer[self.index];
            self.buffer[self.index] = v_out;
            self.sum += v_out;
            self.index = (self.index + 1) % N;
        }
    }

    /// Returns the average voltage at the sensor's output pin (before divider)
    pub fn get_voltage(&self) -> f32 {
        self.sum / (N as f32)
    }

    /// Returns the current in Amperes, relative to the zero level.
    pub fn get_current(&self) -> f32 {
        let v_out = self.get_voltage();
        (v_out - self.zero_offset) / self.sensitivity
    }

    /// Returns the absolute current in Amperes.
    pub fn get_current_abs(&self) -> f32 {
        let current = self.get_current();
        if current < 0.0 { -current } else { current }
    }

    /// Samples for exactly one full buffer cycle (sample_interval_ms * N)
    /// to find the zero-current voltage level.
    pub fn calibrate(&mut self) {
        let mut local_now = self.last_update_ms;
        let end_time = local_now + (self.sample_interval_ms * N as u32);
        while local_now < end_time {
            self.sample(local_now);
            local_now += 1;
        }
        self.last_update_ms = 0;
        self.zero_offset = self.get_voltage();
        rtt_target::rprintln!("Current Calibrated. Zero: {:.3}V", self.zero_offset);
    }
}
