#![no_std]
#![no_main]

mod drivers;
use drivers::m5weight;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{pac, prelude::*};

use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("M5Weight Test Starting...");

    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOF.split();

    // I2C Setup: PB8 (SCL), PB9 (SDA)
    let sda = gpio.pf0.into_alternate_open_drain();
    let scl = gpio.pf1.into_alternate_open_drain();

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::standard(100.kHz()),
        &clocks,
        &mut rcc.apb1,
        1000,
    );

    let mut sensor = M5Weight::new(i2c, DEVICE_DEFAULT_ADDR);

    rprintln!("Probing sensor...");
    if sensor.probe() {
        rprintln!("Sensor found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
    } else {
        rprintln!("Sensor NOT found! Check wiring.");
    }

    // Initialize (sets offset)
    if let Ok(_) = sensor.init() {
        rprintln!("Initialization (Tare) Successful");
    } else {
        rprintln!("Initialization Failed");
    }

    // Set Gap Value
    sensor.set_gap_value(915.1742).unwrap();

    // Read Config
    sensor.query_config();

    rprintln!("Firmware Version: {}", sensor.firmware_version);
    rprintln!("LP Filter: {}", sensor.lp_filter);
    rprintln!("Avg Filter: {}", sensor.avg_filter);
    rprintln!("Gap Value: {:.4}", sensor.gap_value);

    cortex_m::asm::delay(216_000_000 * 10); // 10 second delay
    rprintln!("Starting Measurement Loop...");
    loop {
        // Read generic Data struct
        let data = sensor.run_step();

        rprintln!(
            "Weight: {:.2}, Raw ADC: {}, Filtered(Int): {}",
            data.force,
            data.raw_adc,
            data.force_int
        );

        cortex_m::asm::delay(216_000_000); // ~1s delay
    }
}
