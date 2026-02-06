#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Button Test Starting...");

    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOA.split();

    // I2C Setup: PB8 (SCL), PB9 (SDA)
    let a0 = gpio.pa3.into_pull_up_input();

    let mut state: bool = false;
    rprintln!("Probing button");
    loop {
        if a0.is_high() {
            rprintln!("Low.");
        } else {
            rprintln!("High.");
        }
        cortex_m::asm::delay(216_000_000 / 100);
    }
}
