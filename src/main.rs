#![no_std]
#![no_main]

mod esc;
mod m5weight;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f7xx_hal::{pac, prelude::*};

use crate::esc::EscController;
use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};

const DSHOT_HERTZ: u32 = 150_000;
const GAP_VALUE: f32 = 915.1742;
const CLOCK_CYCLES_PER_SECOND: u32 = 216_000_000;
const MIN_THROTTLE: f32 = 3.0;
const MAX_THROTTLE: f32 = 15.0;

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

fn print_weight<I2C, E>(sensor: &mut M5Weight<I2C>)
where
    I2C: embedded_hal::blocking::i2c::Write<Error = E>
        + embedded_hal::blocking::i2c::WriteRead<Error = E>,
{
    let data = sensor.run_step();
    rprintln!("Weight: {:.2}", data.force);
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Real DShot Initializing (DMA-driven, Single Direction)...");

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();

    // Configure PE9 for DShot: Idle LOW
    let mut pe9_gpio = gpioe.pe9.into_push_pull_output();
    pe9_gpio.set_low();
    // Configure PA3 for bistable button
    let pa3_gpio = gpioa.pa3.into_pull_up_input();
    // I2C setup: PB8 (SCL), PB9 (SDA)
    let sda = gpiof.pf0.into_alternate_open_drain();
    let scl = gpiof.pf1.into_alternate_open_drain();

    rprintln!("Set PE9 LOW, waiting 3 seconds...");
    cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND * 3); // 3 second delay

    let pe9 = pe9_gpio
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::VeryHigh);

    // DShot150 as requested
    let hertz = DSHOT_HERTZ.Hz();
    let mut esc = EscController::new(dp.TIM1, pe9, hertz, &clocks, dp.DMA2);
    rprintln!("Initialized ESC");

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::standard(100.kHz()),
        &clocks,
        &mut rcc.apb1,
        1000,
    );
    let mut sensor = M5Weight::new(i2c, DEVICE_DEFAULT_ADDR);
    if sensor.probe() {
        rprintln!("M5Weight found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
    } else {
        panic!("M5Weight NOT found! Check wiring.");
    }

    arm_esc(&mut esc);
    sensor.init().unwrap();
    sensor.set_gap_value(GAP_VALUE).unwrap();

    let initial_state = pa3_gpio.is_high();

    let mut throttle = MIN_THROTTLE;
    let mut loops = 0;
    loop {
        let button_state = pa3_gpio.is_high();
        if throttle <= MIN_THROTTLE {
            esc.send_throttle(0.0);
        } else {
            esc.send_throttle(throttle);
        }

        if initial_state != button_state {
            if throttle < MAX_THROTTLE {
                throttle += 0.01;
            }
        } else if initial_state == button_state {
            if throttle > MIN_THROTTLE {
                throttle -= 0.01;
            }
        }

        if loops % 1000 == 0 {
            print_weight(&mut sensor);
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
        loops += 1;
    }
}
