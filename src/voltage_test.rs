#![no_std]
#![no_main]

mod drivers;
use drivers::voltage;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC3},
    prelude::*,
};

use crate::drivers::calibration;
use crate::voltage::VoltageSensor;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    // Initialize peripherals
    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    // Configure clocks - simplified for test, but sufficient for ADC
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Configure GPIO C
    let gpiof = dp.GPIOF.split();

    // Configure PF5 as analog input (ADC3 Channel 10)
    let voltage_pin = gpiof.pf5.into_analog();

    // Calibrate ADC to VREFINT
    let vdda = calibration::get_avdd(dp.ADC1, &dp.ADC_COMMON, &mut rcc.apb2, &clocks);
    rprintln!("Calibrated VDDA: {:.3} V", vdda);
    cortex_m::asm::delay(216_000_000 * 3);

    // Initialize ADC3 for the voltage sensor
    let adc3 = Adc::adc3(dp.ADC3, &mut rcc.apb2, &clocks, 12, false);

    let voltage_divider = 11.0;

    let mut sensor =
        VoltageSensor::<ADC3, _, 20>::new(adc3, voltage_pin, voltage_divider, None, 500, vdda);

    rprintln!("Starting Voltage Test on PC0 (ADC3/10)...");

    let mut now_ms = 0;
    loop {
        sensor.sample(now_ms);
        let voltage = sensor.read();

        let battery_status = if voltage < 0.2 {
            "DISCONNECTED"
        } else {
            "CONNECTED"
        };
        if now_ms % 1000 == 0 {
            rprintln!("Voltage: {:.2} V ({})", voltage, battery_status);
        }

        // delay for 200ms
        cortex_m::asm::delay(216_000_000 / 200);
        now_ms += 5;
    }
}
