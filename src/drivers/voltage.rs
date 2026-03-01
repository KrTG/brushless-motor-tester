use embedded_hal::adc::OneShot;
use stm32f7xx_hal::adc::Adc;

const LIPO_DEAD_VOLTAGE: f32 = 2.5;
const MIN_VOLTAGE_PER_CELL: f32 = 3.3; // critical level - panic
const WARNING_VOLTAGE_PER_CELL: f32 = 3.5; // warning level - do not use
const MAX_VOLTAGE_PER_CELL: f32 = 4.35;

pub struct VoltageSensor<ADC, PIN, const N: usize = 30> {
    adc: Adc<ADC>,
    pin: PIN,
    voltage_divider: f32,
    buffer: [f32; N],
    index: usize,
    sum: f32,
    initialized: bool,
    last_update_ms: u32,
    sample_interval_ms: u32,
    battery_s: u8,
    vdda: f32,
}

impl<ADC, PIN, const N: usize> VoltageSensor<ADC, PIN, N>
where
    Adc<ADC>: OneShot<ADC, u16, PIN>,
    PIN: embedded_hal::adc::Channel<ADC>,
{
    pub fn new(
        adc: Adc<ADC>,
        pin: PIN,
        voltage_divider: f32,
        battery_s: Option<u8>,
        sample_interval_ms: u32,
        vdda: f32,
    ) -> Self {
        let mut instance = Self {
            adc,
            pin,
            voltage_divider,
            buffer: [0.0; N],
            index: 0,
            sum: 0.0,
            initialized: false,
            last_update_ms: 0,
            sample_interval_ms,
            battery_s: battery_s.unwrap_or(0),
            vdda,
        };
        instance.sample(0);
        instance
    }

    pub fn sample(&mut self, now_ms: u32) {
        if self.initialized && now_ms - self.last_update_ms < self.sample_interval_ms {
            return;
        }

        self.measure();
        self.last_update_ms = now_ms;

        if self.battery_s == 0 {
            self.guess_battery_cells();
        } else {
            let smoothed_voltage = self.read_per_cell();
            if smoothed_voltage > LIPO_DEAD_VOLTAGE && smoothed_voltage < MIN_VOLTAGE_PER_CELL {
                panic!("Battery voltage is too low!");
            }
        }
    }

    fn measure(&mut self) {
        let sample = loop {
            match self.adc.read(&mut self.pin) {
                Ok(sample) => break sample,
                Err(_) => continue,
            }
        };
        // STM32 - 12 bit ADC
        let val = (sample as f32) * (self.vdda / 4095.0) * self.voltage_divider;

        if !self.initialized {
            for i in 0..N {
                self.buffer[i] = val;
            }
            self.sum = val * (N as f32);
            self.initialized = true;
        } else {
            // Subtract the old value and add the new one
            self.sum -= self.buffer[self.index];
            self.buffer[self.index] = val;
            self.sum += val;

            // Move the index
            self.index = (self.index + 1) % N;
        }
    }

    fn read_internal(&self) -> f32 {
        self.sum / (N as f32)
    }

    pub fn read(&self) -> f32 {
        if self.battery_s == 0 {
            return 0.0;
        }
        self.read_internal()
    }

    pub fn read_per_cell(&self) -> f32 {
        if self.battery_s == 0 {
            return 0.0;
        }
        self.read_internal() / (self.battery_s as f32)
    }

    pub fn is_low(&self) -> bool {
        self.battery_s != 0 && self.read_per_cell() < WARNING_VOLTAGE_PER_CELL
    }

    pub fn is_unstable(&self) -> bool {
        let mean = self.read_internal();
        let mut sum_abs_diff = 0.0;
        for &val in &self.buffer {
            sum_abs_diff += (val - mean).abs();
        }
        let avg_diff = sum_abs_diff / (N as f32);
        // A threshold of 0.15V average deviation is robust against occasional
        // ADC noise but will trigger if the voltage is trending or has high ripple.
        avg_diff > 0.15
    }

    fn guess_battery_cells(&mut self) {
        if self.battery_s != 0 {
            return;
        }
        if self.is_unstable() {
            return;
        }
        let voltage = self.read_internal();
        if voltage <= LIPO_DEAD_VOLTAGE {
            self.battery_s = 0;
        } else if voltage < MAX_VOLTAGE_PER_CELL {
            self.battery_s = 1;
        } else if voltage < MAX_VOLTAGE_PER_CELL * 2.0 {
            self.battery_s = 2;
        } else if voltage < MAX_VOLTAGE_PER_CELL * 3.0 {
            self.battery_s = 3;
        } else if voltage < MAX_VOLTAGE_PER_CELL * 4.0 {
            self.battery_s = 4;
        } else if voltage < MAX_VOLTAGE_PER_CELL * 6.0 {
            self.battery_s = 6;
        } else {
            self.battery_s = 8;
        }
    }
}
