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
    let pe9 = gpio
        .pe9
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain(); // TIM1_CH1

    rprintln!("Initial delay of 10 seconds.");
    cortex_m::asm::delay(216_000_000 * 10);
    rprintln!("Initial delay done.");

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

    rprintln!("Initialized");
    cortex_m::asm::delay(216_000_000 * 5);

    rprintln!("Start beeping sequence");
    for _ in 0..20 {
        rprintln!("Beep");
        esc.beep(5);
        // Time delay: 100ms
        cortex_m::asm::delay(216_000_000 / 10);
    }
    rprintln!("Done beeping");

    // Transition to active control
    esc.set_armed(true);
    //esc.request_info(); // Request ESC info (and clear warning)
    //let info = esc.get_last_telemetry();
    //rprintln!("ESC Info Received: {:#06x}", info);

    loop {
        esc.send_throttle(10.0, false);

        //let rpm = esc.get_rpm();
        //rprintln!("Motor RPM: {}", rpm);

        cortex_m::asm::delay(216_000_000 / 20);
    }
}
