#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::Text,
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Screen test starting...");

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

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Draw a simple rectangle to test the screen (since rust.raw is missing)
    Rectangle::new(Point::new(0, 0), Size::new(127, 63))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    Text::new("Hello Rust!", Point::new(20, 30), text_style)
        .draw(&mut display)
        .unwrap();

    Text::new("STM32F767 OLED", Point::new(20, 45), text_style)
        .draw(&mut display)
        .unwrap();

    rprintln!("Screen initialized and test pattern drawn.");

    display.flush().unwrap();

    loop {}
}
