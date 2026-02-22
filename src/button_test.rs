#![no_std]
#![no_main]

use cortex_m_rt::entry;

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{
    gpio::{Pin, PullUp},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Button Test Starting...");

    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let gpiof = dp.GPIOF.split();

    // Buttons: PA3 (B1), PC0 (M2), PC3 (M3), PF3 (M4)
    let button4 = gpioa.pa3.into_pull_up_input();
    let button3 = gpioc.pc0.into_pull_up_input();
    let button2 = gpioc.pc3.into_pull_up_input();
    let button1 = gpiof.pf3.into_pull_up_input();

    // Store references to the buttons in an array using trait objects.
    // Each pin is a unique type, so we need &dyn InputPin to put them in the same array.
    let buttons: [&dyn InputPin<Error = core::convert::Infallible>; 4] =
        [&button1, &button2, &button3, &button4];

    rprintln!("Probing buttons (H = High/Idle, L = Low/Pressed)...");
    loop {
        rtt_target::rprint!("|");
        for (i, button) in buttons.iter().enumerate() {
            let label = match i {
                0 => "1",
                1 => "2",
                2 => "3",
                3 => "4",
                _ => "?",
            };

            // is_high() on the trait returns Result, so we unwrap() since GPIO is infallible.
            let state = if button.is_high().unwrap() { '1' } else { '0' };
            rtt_target::rprint!("{} : {} | ", label, state);
        }
        rtt_target::rprintln!(""); // New line after checking all buttons

        cortex_m::asm::delay(216_000_000); // Check ~10 times per second
    }
}
