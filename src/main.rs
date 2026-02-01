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
    let mut pe9_test = gpio.pe9.into_push_pull_output();

    rprintln!("--- HARDWARE TEST ---");
    cortex_m::asm::delay(216_000_000 * 10);
    rprintln!("PE9 LOW (Driven to GND) for 5s... Should be 0V");
    pe9_test.set_low();
    cortex_m::asm::delay(216_000_000 * 10);
    rprintln!("PE9 HIGH (Released to Pull-up) for 5s... Should be ~3.3V");
    pe9_test.set_high();
    cortex_m::asm::delay(216_000_000 * 10);
    rprintln!("--- TEST COMPLETE ---");

    let pe9 = pe9_test
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::VeryHigh);

    // DShot300 is the goal.
    // The HAL calculates timer ticks for 150kHz as 1440.
    // So if we request 600kHz, it should calculate ~360?
    // Wait. 216MHz / 1440 = 150kHz.
    // So current 150,000.Hz() -> 150kHz.
    // We want 300kHz. So we should request 300,000.Hz().
    // Wait, previous code was 150k?
    let hertz = 300_000.Hz();
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

    // Safety Print: Check max_duty calculation
    // If it's 720 for 300kHz/216MHz, we are good.
    // If it's 360, we have accidentally halved the clock.
    rprintln!("ESC Max Duty (ARR): {}", esc.max_duty());

    rprintln!("Initialized");
    cortex_m::asm::delay(216_000_000 * 2);

    rprintln!("Arming ESC (Sending zero throttle)...");
    for _ in 0..2000 {
        esc.send_throttle(0.0, false);
        cortex_m::asm::delay(216_000_000 / 1000); // 1ms rate
    }

    // NOW we are ready to run
    esc.set_armed(true);

    rprintln!("Ramping up throttle...");
    // Slow ramp from 0% to 15%
    for i in 0..150 {
        let throttle = i as f32 / 10.0; // 0.0, 0.1 ... 15.0
        esc.send_throttle(throttle, false);
        cortex_m::asm::delay(216_000_000 / 500); // 2ms per step
    }

    rprintln!("Holding 15%...");
    loop {
        esc.send_throttle(15.0, false);
        cortex_m::asm::delay(216_000_000 / 500);
    }
}
