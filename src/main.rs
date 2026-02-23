#![no_std]
#![no_main]

mod esc;
mod input;
mod m5weight;
mod ui;
mod voltage;

use crate::input::Button;

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC3},
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

    let cp = pac::CorePeripherals::take().expect("Failed to take CorePeripherals");
    let dp = pac::Peripherals::take().expect("Failed to take Peripherals");

    // Initialize Hardware Timer (DWT)
    let mut dcb = cp.DCB;
    let mut dwt = cp.DWT;

    // Unlock DWT access (Required on many STM32F7 chips)
    unsafe {
        // LAR is at offset 0xFB0 in DWT/ITM
        let lar_ptr = (0xE0001FB0 as *mut u32);
        lar_ptr.write_volatile(0xC5ACCE55);
    }

    dcb.enable_trace();
    dwt.enable_cycle_counter();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Configure PE9 for DShot: Idle LOW
    let mut esc_data_pin = gpioe.pe9.into_push_pull_output();
    // Configure PA3 for bistable button
    let mut button_start = Button::new(gpioa.pa3.into_pull_up_input(), 100);
    let mut button_down = Button::new(gpioc.pc0.into_pull_up_input(), 100);
    let mut button_right = Button::new(gpioc.pc3.into_pull_up_input(), 100);
    let mut button_left = Button::new(gpiof.pf3.into_pull_up_input(), 100);
    // I2C setup: PF0 (SDA), PF1 (SCL) on AF4
    // High speed ensures sharp signal edges for 400kHz+
    let sda = gpiof
        .pf0
        .into_alternate::<4>()
        .set_speed(stm32f7xx_hal::gpio::Speed::High)
        .set_open_drain();
    let scl = gpiof
        .pf1
        .into_alternate::<4>()
        .set_speed(stm32f7xx_hal::gpio::Speed::High)
        .set_open_drain();
    // Configure PC0 for voltage sensor
    let voltage_pin = gpiof.pf5.into_analog();

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::fast(400.kHz()),
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
    esc_data_pin.set_low();
    for i in 0..3000 {
        if i % 100 == 0 {
            ui.display_loading();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
    }

    let esc_data_pin = esc_data_pin
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::VeryHigh);

    // DShot150 as requested
    let hertz = DSHOT_HERTZ.Hz();
    let mut esc = EscController::new(dp.TIM1, esc_data_pin, hertz, &clocks, dp.DMA2);
    rprintln!("Initialized ESC");

    let mut sensor = M5Weight::new(bus.acquire_i2c(), DEVICE_DEFAULT_ADDR);
    if sensor.probe() {
        rprintln!("M5Weight found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
    } else {
        rprintln!("M5Weight NOT found! Check wiring. Proceeding anyway...");
    }

    let adc3 = Adc::<ADC3>::adc3(dp.ADC3, &mut rcc.apb2, &clocks, 12, false);
    let mut voltage_sensor = VoltageSensor::<_, _, 20>::new(adc3, voltage_pin, 11.0, None, 500);

    arm_esc(&mut esc, &mut ui);

    sensor.init().unwrap();
    sensor.set_gap_value(GAP_VALUE).unwrap();
    rprintln!("Gap value set to {}", GAP_VALUE);

    let sysclk_hz = clocks.sysclk().raw();
    let ticks_per_ms = sysclk_hz / 1000;
    rprintln!("Ticks per ms: {}", ticks_per_ms);

    let mut last_ticks = dwt.cyccnt.read();
    let mut accumulator: u32 = 0;
    let mut time_ms: u32 = 0;

    button_start.update(0);
    rprintln!("Button start state: {}", button_start.is_pressed());
    let mut initial_state = button_start.is_pressed();
    let mut timer_start_ms: Option<u32> = None;

    let mut throttle = MIN_THROTTLE;
    let mut last_ui_ms = 0;
    let mut last_print_ms = 0;
    let mut last_esc_ms = 0;
    let mut ramp_up_ms = 0;

    loop {
        let current_ticks = dwt.cyccnt.read();
        let delta = current_ticks.wrapping_sub(last_ticks);
        last_ticks = current_ticks;

        accumulator += delta;
        while accumulator >= ticks_per_ms {
            accumulator -= ticks_per_ms;
            time_ms += 1;
        }

        button_start.update(time_ms);
        let button_start_state = button_start.is_pressed();
        let button_down_pulses = button_down.update(time_ms);
        let button_right_pulses = button_right.update(time_ms);
        let button_left_pulses = button_left.update(time_ms);
        voltage_sensor.sample(time_ms);

        if time_ms - last_esc_ms >= 7 {
            last_esc_ms = time_ms;
            if throttle <= MIN_THROTTLE {
                esc.send_throttle(0.0);
            } else {
                esc.send_throttle(throttle);
            }
        }

        if (voltage_sensor.is_low() || initial_state == button_start_state)
            && throttle <= MIN_THROTTLE
        {
            if button_down_pulses > 0 {
                ui.down();
            } else if button_right_pulses > 0 {
                ui.right();
            } else if button_left_pulses > 0 {
                ui.left();
            }
        } else if (initial_state != button_start_state && throttle >= ui.throttle_setpoint) {
            if ui.timer_setpoint > 0.0 {
                if let Some(start_time) = timer_start_ms {
                    if time_ms - start_time >= (ui.timer_setpoint * 1000.0) as u32 {
                        initial_state = button_start_state;
                        timer_start_ms = None;
                    }
                } else {
                    timer_start_ms = Some(time_ms);
                }
            }
        }

        if time_ms - ramp_up_ms >= 50 {
            ramp_up_ms = time_ms;
            if voltage_sensor.is_low() || initial_state == button_start_state {
                timer_start_ms = None;
                if throttle > MIN_THROTTLE {
                    throttle -= if throttle < 25.0 { 0.6 } else { 3.0 };
                }
            } else {
                if throttle < ui.throttle_setpoint {
                    throttle += if throttle < 25.0 { 0.6 } else { 3.0 };
                }
            }
        }

        if time_ms - last_ui_ms >= 211 {
            last_ui_ms = time_ms;
            if throttle >= ui.throttle_setpoint {
                if let Ok(weight_str) = sensor.get_weight_string() {
                    let time_left = timer_start_ms.map(|start_time| {
                        let elapsed = (time_ms - start_time) as f32 / 1000.0;
                        (ui.timer_setpoint - elapsed).max(0.0)
                    });
                    ui.display_force(
                        weight_str,
                        voltage_sensor.read(),
                        voltage_sensor.read_per_cell(),
                        time_left,
                    );
                }
            } else {
                if throttle <= MIN_THROTTLE {
                    ui.display_options(voltage_sensor.read(), voltage_sensor.read_per_cell());
                } else {
                    ui.display_throttle(
                        throttle,
                        voltage_sensor.read(),
                        voltage_sensor.read_per_cell(),
                    );
                }
            }
        }

        if time_ms - last_print_ms >= 1291 {
            last_print_ms = time_ms;
            print_weight(&mut sensor);
            rprintln!(
                "Voltage: {:.2} V ({:.2} V per cell)",
                voltage_sensor.read(),
                voltage_sensor.read_per_cell()
            );
            rprintln!("Delta: {}", delta);
        }
    }
}
