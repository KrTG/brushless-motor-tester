#![no_std]
#![no_main]

mod m5weight;

use cortex_m_rt::{ExceptionFrame, entry, exception};
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use heapless::String;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{pac, prelude::*};

use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};

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
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

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

    let text_style = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

    loop {
        // Clear the buffer
        display.clear(BinaryColor::Off).unwrap();

        // Draw a border
        Rectangle::new(Point::new(0, 0), Size::new(127, 63))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        // Try to read weight string
        if !sensor.disconnected {
            if let Ok(weight_bytes) = sensor.get_weight_string() {
                // Convert bytes to string safely
                let weight_str = core::str::from_utf8(&weight_bytes)
                    .unwrap_or("Invalid Text")
                    .trim_matches('\0')
                    .trim();

                // Concatenate "F: " and weight_str using heapless
                let mut display_str = String::<32>::new();
                let _ = display_str.push_str("F: ");
                let _ = display_str.push_str(weight_str);
                let _ = display_str.push_str("N");

                // Display the combined weight string
                Text::new(&display_str, Point::new(10, 35), text_style)
                    .draw(&mut display)
                    .unwrap();

                rprintln!("Current Weight: {}", weight_str);
            }
        } else {
            Text::new("Sensor Offline", Point::new(10, 30), text_style)
                .draw(&mut display)
                .unwrap();

            // Try to re-probe if disconnected
            sensor.probe();
        }

        // Send buffer to the physical screen
        display.flush().unwrap();

        // Slow down the loop a bit
        cortex_m::asm::delay(216_000_000 / 10); // ~100ms
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}
