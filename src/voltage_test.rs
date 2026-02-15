#![no_std]
#![no_main]

mod voltage;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC1},
    prelude::*,
};

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
    let gpioc = dp.GPIOC.split();

    // Configure PC0 as analog input (ADC1 Channel 10)
    let voltage_pin = gpioc.pc0.into_analog();

    // Initialize ADC1
    let adc1 = Adc::<ADC1>::adc1(dp.ADC1, &mut rcc.apb2, &clocks, 12, false);

    let voltage_divider = 11.0;

    // Window duration: 15 seconds (15,000 ms)
    let mut sensor = VoltageSensor::<_, 30>::new(adc1, voltage_pin, voltage_divider, 15_000);

    rprintln!("Starting Voltage Test on PC0 (ADC1/10)...");

    let mut now_ms = 0;
    loop {
        let voltage = sensor.read(now_ms);
        rprintln!("Time Index: {} ms | Voltage: {:.3} V", now_ms, voltage);

        // Delay for 500ms
        cortex_m::asm::delay(216_000_000 / 2);
        now_ms += 500;
    }
}
