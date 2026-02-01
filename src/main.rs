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
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Disable D-Cache to rule out coherency issues
    cp.SCB.disable_dcache(&mut cp.CPUID);
    // Enable I-Cache for performance
    cp.SCB.enable_icache();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOE.split();

    // Configure PE9 for DShot: Pull-up (idle=high), Low speed (reduces ringing, matches Betaflight for F7)
    let mut pe9_gpio = gpio.pe9.into_push_pull_output();
    pe9_gpio.set_high(); // Set idle level HIGH before switching to alternate function

    let pe9 = pe9_gpio
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::Low)
        .internal_pull_up(true);

    // DShot300 is the goal.
    let hertz = 300_000.Hz();
    let motor_poles = 12;

    let mut esc = EscController::new(
        dp.TIM1,
        pe9, // pe9
        hertz,
        motor_poles,
        &clocks,
        dp.DMA2,
        cp.SCB,
        true, // is_blocking
    )
    .expect("Failed to initialize ESC");

    // Safety Print
    rprintln!("ESC Max Duty (ARR): {}", esc.max_duty());

    rprintln!("Initialized");
    cortex_m::asm::delay(216_000_000 * 2);

    rprintln!("Arming ESC (Sending zero throttle)...");
    for _ in 0..4000 {
        esc.send_throttle(0.0, false);
        cortex_m::asm::delay(216_000_000 / 4000); // 4kHz rate
    }

    // NOW we are ready to run
    esc.set_armed(true);

    rprintln!("Ramping up throttle...");
    // Slow ramp from 0% to 15%
    for i in 0..600 {
        let throttle = i as f32 / 40.0; // 0.0 ... 15.0 approx
        esc.send_throttle(throttle, false);
        cortex_m::asm::delay(216_000_000 / 4000); // 4kHz rate
    }

    rprintln!("Holding 15%...");
    loop {
        esc.send_throttle(15.0, false);
        cortex_m::asm::delay(216_000_000 / 4000); // 4kHz rate
    }
}
