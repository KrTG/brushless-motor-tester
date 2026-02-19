#![no_std]
#![no_main]

mod esc;
mod m5weight;
mod ui;
mod voltage;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC1},
    prelude::*,
};

use crate::esc::EscController;
use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};
use crate::ui::Ui;
use crate::voltage::VoltageSensor;

const DSHOT_HERTZ: u32 = 150_000;
const GAP_VALUE: f32 = 915.1742;
const CLOCK_CYCLES_PER_SECOND: u32 = 216_000_000;
const MIN_THROTTLE: f32 = 3.0;
const MAX_THROTTLE: f32 = 28.0;

fn arm_esc<DI, SIZE>(esc: &mut EscController, ui: &mut Ui<DI, SIZE>)
where
    DI: ssd1306::prelude::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    rprintln!("Arming ESC...");
    rprintln!("Sending MotorStop...");
    for i in 0..3000 {
        esc.send_stop();
        if i % 100 == 0 {
            ui.display_loading();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
    }
    rprintln!("Sending Throttle 0...");
    for i in 0..3000 {
        esc.send_throttle(0.0);
        if i % 100 == 0 {
            ui.display_loading();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
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
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Configure PE9 for DShot: Idle LOW
    let mut pe9_gpio = gpioe.pe9.into_push_pull_output();
    // Configure PA3 for bistable button
    let pa3_gpio = gpioa.pa3.into_pull_up_input();
    // I2C setup: PB8 (SCL), PB9 (SDA)
    let sda = gpiof.pf0.into_alternate_open_drain();
    let scl = gpiof.pf1.into_alternate_open_drain();
    // Configure PC0 for voltage sensor
    let voltage_pin = gpioc.pc0.into_analog();

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::standard(100.kHz()),
        &clocks,
        &mut rcc.apb1,
        1000,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let interface = I2CDisplayInterface::new(bus.acquire_i2c());
    let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    let mut ui = Ui::new(display);
    ui.init().unwrap();

    rprintln!("Set PE9 LOW, waiting 3 seconds...");
    pe9_gpio.set_low();
    for i in 0..3000 {
        if i % 100 == 0 {
            ui.display_loading();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
    }

    let pe9 = pe9_gpio
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::VeryHigh);

    // DShot150 as requested
    let hertz = DSHOT_HERTZ.Hz();
    let mut esc = EscController::new(dp.TIM1, pe9, hertz, &clocks, dp.DMA2);
    rprintln!("Initialized ESC");

    let mut sensor = M5Weight::new(bus.acquire_i2c(), DEVICE_DEFAULT_ADDR);
    if sensor.probe() {
        rprintln!("M5Weight found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
    } else {
        rprintln!("M5Weight NOT found! Check wiring. Proceeding anyway...");
    }

    let adc1 = Adc::<ADC1>::adc1(dp.ADC1, &mut rcc.apb2, &clocks, 12, false);
    let mut voltage_sensor = VoltageSensor::<_, 20>::new(adc1, voltage_pin, 11.0, None, 500);

    arm_esc(&mut esc, &mut ui);

    sensor.init().unwrap();
    sensor.set_gap_value(GAP_VALUE).unwrap();

    let initial_state = pa3_gpio.is_high();

    let mut throttle = MIN_THROTTLE;
    let mut loops = 0;
    let mut time_ms = 0;
    loop {
        let button_state = pa3_gpio.is_high();
        voltage_sensor.sample(time_ms);

        if throttle <= MIN_THROTTLE {
            esc.send_throttle(0.0);
        } else {
            esc.send_throttle(throttle);
        }
        if voltage_sensor.is_low() || initial_state == button_state {
            if throttle > MIN_THROTTLE {
                throttle -= 0.01;
            }
        } else {
            if throttle < MAX_THROTTLE {
                throttle += 0.01;
            }
        }

        if loops % 100 == 0 {
            if throttle >= MAX_THROTTLE {
                if let Ok(weight_str) = sensor.get_weight_string() {
                    ui.display_force(
                        weight_str,
                        voltage_sensor.read(),
                        voltage_sensor.read_per_cell(),
                    );
                }
            } else {
                if throttle <= MIN_THROTTLE {
                    ui.display_thrust(0.0, voltage_sensor.read(), voltage_sensor.read_per_cell());
                } else {
                    ui.display_thrust(
                        throttle,
                        voltage_sensor.read(),
                        voltage_sensor.read_per_cell(),
                    );
                }
            }
        }

        if loops % 1000 == 0 {
            print_weight(&mut sensor);
            rprintln!(
                "Voltage: {:.2} V ({:.2} V per cell)",
                voltage_sensor.read(),
                voltage_sensor.read_per_cell()
            );
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
        loops += 1;
        time_ms += 1;
    }
}
