#![no_std]
#![no_main]

mod esc;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{pac, prelude::*};

use crate::esc::EscController;

fn arm_esc(esc: &mut EscController) {
    rprintln!("Arming ESC...");
    rprintln!("Sending MotorStop...");
    for _ in 0..3000 {
        esc.send_stop();
        cortex_m::asm::delay(216_000_000 / 1000);
    }
    rprintln!("Sending Throttle 0...");
    for _ in 0..3000 {
        esc.send_throttle(0.0);
        cortex_m::asm::delay(216_000_000 / 1000);
    }
}

fn disarm_esc(esc: &mut EscController) {
    rprintln!("Disarming ESC...");
    rprintln!("Sending Throttle 0...");
    for _ in 0..3000 {
        esc.send_throttle(0.0);
        cortex_m::asm::delay(216_000_000 / 1000);
    }
    rprintln!("Sending MotorStop...");
    for _ in 0..3000 {
        esc.send_stop();
        cortex_m::asm::delay(216_000_000 / 1000);
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Real DShot Initializing (DMA-driven, Single Direction)...");

    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOE.split();

    // Configure PE9 for DShot: Idle LOW
    let mut pe9_gpio = gpio.pe9.into_push_pull_output();
    pe9_gpio.set_low();

    rprintln!("Set PE9 LOW, waiting 3 seconds...");
    cortex_m::asm::delay(216_000_000 * 3); // 3 second delay

    let pe9 = pe9_gpio
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::VeryHigh);

    // DShot150 as requested
    let hertz = 150_000.Hz();
    let mut esc = EscController::new(dp.TIM1, pe9, hertz, &clocks, dp.DMA2);
    rprintln!("Initialized");

    arm_esc(&mut esc);

    rprintln!("Ramping up throttle (5% to 25%)...");
    for i in 0..2000 {
        let throttle = 5.0 + (i as f32 / 100.0);
        esc.send_throttle(throttle);
        cortex_m::asm::delay(216_000_000 / 1000);
    }

    rprintln!("Holding 25% for 2 seconds...");
    for _ in 0..2000 {
        esc.send_throttle(25.0);
        cortex_m::asm::delay(216_000_000 / 1000);
    }

    rprintln!("Stopping (Ramp Down)...");
    for i in (0..2000).rev() {
        let throttle = 5.0 + (i as f32 / 100.0);
        esc.send_throttle(throttle);
        cortex_m::asm::delay(216_000_000 / 1000);
    }

    disarm_esc(&mut esc);
    rprintln!("Done.");

    loop {}
}
