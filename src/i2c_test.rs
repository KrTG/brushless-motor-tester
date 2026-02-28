#![no_std]
#![no_main]

mod drivers;
use drivers::m5weight;
use drivers::ui;

use cortex_m_rt::{ExceptionFrame, entry, exception};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{pac, prelude::*};

use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};
use crate::ui::Ui;

const GRAVITY: f32 = 9.80665;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("I2C Shared Bus Test (Display + Weight Sensor)...");

    let dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpiof = dp.GPIOF.split();

    // I2C Setup: PF0 (SDA), PF1 (SCL)
    let sda = gpiof.pf0.into_alternate_open_drain();
    let scl = gpiof.pf1.into_alternate_open_drain();

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::standard(100.kHz()),
        &clocks,
        &mut rcc.apb1,
        1000,
    );

    // Create the shared bus manager
    let bus = shared_bus::BusManagerSimple::new(i2c);

    // Initialize the display using a proxy from the shared bus
    let interface = I2CDisplayInterface::new(bus.acquire_i2c());
    let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    let mut ui = Ui::new(display);
    ui.init().unwrap();

    // Show loading animation for ~2 seconds
    for _ in 0..100 {
        ui.display_loading();
        cortex_m::asm::delay(216_000_000 / 50); // ~50 FPS
    }

    // Initialize the weight sensor using another proxy from the shared bus
    let mut sensor = M5Weight::new(bus.acquire_i2c(), DEVICE_DEFAULT_ADDR);

    // Probe the sensor
    if sensor.probe() {
        rprintln!("M5Weight found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
        sensor.init().unwrap();
        // Set a default gap value (calibration)
        sensor.set_gap_value(915174.2 / GRAVITY).unwrap();
    } else {
        rprintln!("M5Weight NOT found!");
    }

    loop {
        // Try to read weight string
        if !sensor.disconnected {
            if let Ok(weight_str) = sensor.get_weight_string() {
                ui.display_sensor_readings(weight_str, 0.0, 0.0, 0.0, None);
            }
        } else {
            ui.display_offline();
            sensor.probe();
        }
        cortex_m::asm::delay(216_000_000 / 10); // ~100ms
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
