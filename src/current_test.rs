#![no_std]
#![no_main]

mod drivers;
use drivers::current;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC3},
    prelude::*,
};

use crate::current::CurrentSensor;
use crate::drivers::calibration;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Current Sensor Test Starting...");

    // Initialize peripherals
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    // Configure clocks (consistent with main.rs)
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Configure GPIO F
    let gpiof = dp.GPIOF.split();

    // Configure PF10 as analog input (ADC3 Channel 8)
    let current_pin = gpiof.pf10.into_analog();

    // Calibrate ADC to VREFINT
    let vdda = calibration::get_avdd(dp.ADC1, &dp.ADC_COMMON, &mut rcc.apb2, &clocks);
    rprintln!("Calibrated VDDA: {:.3} V", vdda);

    // Initialize ADC3
    let adc3 = Adc::<ADC3>::adc3(dp.ADC3, &mut rcc.apb2, &clocks, 12, false);

    // Sensor characteristics
    let voltage_divider = 2.0;
    let sensitivity_mv_a = 66.0; // ACS712-30A sensitivity
    let sample_interval_ms = 25; // Sample at 40Hz for test

    let mut sensor = CurrentSensor::<_, _, 40>::new(
        adc3,
        current_pin,
        voltage_divider,
        sensitivity_mv_a,
        sample_interval_ms,
        vdda,
    );

    rprintln!(
        "Calibrating Current Sensor (Sampling for {}ms)...",
        sample_interval_ms * 40
    );
    sensor.calibrate();
    rprintln!(
        "Calibration Done! Zero Level: {:.3} V",
        sensor.get_voltage()
    );

    rprintln!("Starting Current Test loop...");
    let mut now_ms = (sample_interval_ms * 40) + 1; // Start loop after calibration period

    loop {
        sensor.sample(now_ms);

        let current = sensor.get_current();
        let sensor_voltage = sensor.get_voltage();

        rprintln!("Current: {:.2} A (V_out: {:.3} V)", current, sensor_voltage);

        // delay for roughly 200ms
        cortex_m::asm::delay(216_000_000 / 5);
    }
}
