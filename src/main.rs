#![no_std]
#![no_main]

mod esc;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{pac, prelude::*};

use crate::esc::EscController;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Real DShot Initializing (DMA-driven)...");

    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOE.split();
    let pe9 = gpio.pe9.into_alternate(); // TIM1_CH1

    // 150_000.Hz() returns the correct HertzU32 type
    let hertz = 150_000.Hz();
    let motor_poles = 12;

    let mut esc = EscController::new(
        dp.TIM1,
        pe9, // pe9
        hertz,
        motor_poles,
        &clocks,
        dp.DMA2,
        true, // is_blocking
    )
    .expect("Failed to initialize ESC");

    for _ in 0..20 {
        rprintln!("Beep");
        esc.beep(2);
        // Time delay: 100ms
        cortex_m::asm::delay(216_000_000 / 10 * 4);
    }
    panic!("Done beeping");

    esc.set_armed(true);
    loop {
        // Simple 10% throttle with telemetry request
        esc.send_throttle(10.0, true);

        let rpm = esc.get_rpm();
        rprintln!("Motor RPM: {}", rpm);

        // ~100ms delay between throttle updates
        cortex_m::asm::delay(216_000_000 / 10);
    }
}
