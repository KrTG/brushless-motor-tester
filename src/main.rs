#![no_std]
#![no_main]

mod drivers;
use drivers::current;
use drivers::esc;
use drivers::input;
use drivers::m5weight;
use drivers::ui;
use drivers::voltage;

use crate::drivers::ui::DisplayedUi;
use crate::drivers::ui::Setpoint;
use crate::input::Button;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};
use stm32f7xx_hal::{
    adc::Adc,
    pac::{self, ADC3, interrupt},
    prelude::*,
};

use crate::drivers::calibration;
use crate::esc::EscController;
use crate::m5weight::{DEVICE_DEFAULT_ADDR, M5Weight};
use crate::ui::Ui;
use crate::voltage::VoltageSensor;
use stm32f7xx_hal::gpio::{
    Input, PullUp,
    gpioc::{PC0, PC3},
    gpiof::PF3,
};

type ButtonDown = Button<PC0<Input<PullUp>>>;
type ButtonRight = Button<PC3<Input<PullUp>>>;
type ButtonLeft = Button<PF3<Input<PullUp>>>;

struct ButtonSet {
    down: ButtonDown,
    right: ButtonRight,
    left: ButtonLeft,
}

const DSHOT_HERTZ: u32 = 150_000;
const GAP_VALUE_GRAMS: f32 = 915.1742;
const CLOCK_CYCLES_PER_SECOND: u32 = 216_000_000;
const MIN_THROTTLE: f32 = 3.0;

static G_ESC: Mutex<RefCell<Option<EscController>>> = Mutex::new(RefCell::new(None));
static G_THROTTLE: Mutex<RefCell<f32>> = Mutex::new(RefCell::new(MIN_THROTTLE));
static G_BUTTONS: Mutex<RefCell<Option<ButtonSet>>> = Mutex::new(RefCell::new(None));

fn arm_esc<DI, SIZE>(esc: &mut EscController, ui: &mut Ui<DI, SIZE>)
where
    DI: ssd1306::prelude::WriteOnlyDataCommand,
    SIZE: ssd1306::prelude::DisplaySize,
{
    rprintln!("Arming ESC...");
    rprintln!("Sending MotorStop...");
    ui.set_loading();
    for i in 0..2000 {
        esc.send_stop(false);
        if i % 100 == 0 {
            ui.update(0.0, 0.0, 0.0, 0.0, None, 0.0);
            ui.render_full();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
    }
    rprintln!("Sending Throttle 0...");
    for i in 0..1000 {
        esc.send_throttle(0.0, false);
        if i % 100 == 0 {
            ui.update(0.0, 0.0, 0.0, 0.0, None, 0.0);
            ui.render_full();
        }
        cortex_m::asm::delay(CLOCK_CYCLES_PER_SECOND / 1000);
    }
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
        let lar_ptr = 0xE0001FB0 as *mut u32;
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
    let mut button_start = Button::new(gpioa.pa3.into_pull_up_input(), 25);
    let mut button_down = Button::new(gpioc.pc0.into_pull_up_input(), 25);
    let mut button_right = Button::new(gpioc.pc3.into_pull_up_input(), 25);
    let mut button_left = Button::new(gpiof.pf3.into_pull_up_input(), 25);
    // PC2 for current sensor (ADC1)
    let current_pin = gpioc.pc2.into_analog();
    // I2C setup: PF0 (SDA), PF1 (SCL) on AF4
    // High speed ensures sharp signal edges for 400kHz+
    let sda = gpiof
        .pf0
        .into_alternate::<4>()
        .set_speed(stm32f7xx_hal::gpio::Speed::High)
        .internal_pull_up(true)
        .set_open_drain();

    let scl = gpiof
        .pf1
        .into_alternate::<4>()
        .set_speed(stm32f7xx_hal::gpio::Speed::High)
        .internal_pull_up(true)
        .set_open_drain();

    // Configure PF5 for voltage sensor
    let voltage_pin = gpiof.pf5.into_analog();

    // Let I2C bus settle
    cortex_m::asm::delay(216_000 * 500);

    let i2c = stm32f7xx_hal::i2c::BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::fast(400.kHz()),
        &clocks,
        &mut rcc.apb1,
        1_000_000,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let interface = I2CDisplayInterface::new(bus.acquire_i2c());
    let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    let mut ui = Ui::new(display);
    ui.init().unwrap();

    rprintln!("Set PE9 LOW, waiting 3 seconds...");
    ui.set_loading();
    esc_data_pin.set_low();
    for i in 0..3000 {
        if i % 100 == 0 {
            ui.update(0.0, 0.0, 0.0, 0.0, None, 0.0);
            ui.render_full();
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

    let mut weight_sensor = M5Weight::new(bus.acquire_i2c(), DEVICE_DEFAULT_ADDR);
    if weight_sensor.probe() {
        rprintln!("M5Weight found at address 0x{:02X}", DEVICE_DEFAULT_ADDR);
    } else {
        rprintln!("M5Weight NOT found! Check wiring. Proceeding anyway...");
    }

    // Calibrate ADC to VREFINT and retrieve ADC1
    let (vdda, adc1) = calibration::get_avdd(dp.ADC1, &dp.ADC_COMMON, &mut rcc.apb2, &clocks);
    rprintln!("Calibrated VDDA: {:.3} V", vdda);

    let adc3 = Adc::<ADC3>::adc3(dp.ADC3, &mut rcc.apb2, &clocks, 12, false);
    let mut voltage_sensor =
        VoltageSensor::<_, _, 20>::new(adc3, voltage_pin, 11.0, None, 100, vdda, ui.min_voltage);

    let mut current_sensor = current::CurrentSensor::<_, _, 150>::new(
        adc1,
        current_pin,
        2.0,  // voltage divider
        66.0, // sensitivity mv/a (ACS712-30A)
        10,   // sample interval ms
        vdda,
    );

    rprintln!("Calibrating Current Sensor...");
    current_sensor.calibrate();
    rprintln!(
        "Current Calibration Done! Zero: {:.3}V",
        current_sensor.get_voltage()
    );

    arm_esc(&mut esc, &mut ui);

    let buttons = ButtonSet {
        down: button_down,
        right: button_right,
        left: button_left,
    };

    // After arming, move ESC and buttons to global static and start interrupt-driven updates
    cortex_m::interrupt::free(|cs| {
        *G_ESC.borrow(cs).borrow_mut() = Some(esc);
        *G_BUTTONS.borrow(cs).borrow_mut() = Some(buttons);
    });

    rprintln!("Starting interrupt-driven updates (TIM3, 7ms interval, ESC + Buttons)...");
    let mut esc_timer = dp.TIM3.counter_hz(&clocks);
    esc_timer.start(143.Hz()).unwrap(); // ~7ms interval
    esc_timer.listen(stm32f7xx_hal::timer::Event::Update);

    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32f7xx_hal::pac::Interrupt::TIM3);
    }

    match weight_sensor.init() {
        Ok(_) => rprintln!("Weight sensor initialized"),
        Err(e) => rprintln!("Failed to initialize weight sensor: {:?}", e),
    }

    match weight_sensor.set_gap_value(GAP_VALUE_GRAMS) {
        Ok(_) => rprintln!("Gap value set to {}", GAP_VALUE_GRAMS),
        Err(e) => rprintln!("Failed to set gap value: {:?}", e),
    }

    let sysclk_hz = clocks.sysclk().raw();
    let ticks_per_ms = sysclk_hz / 1000;
    rprintln!("Ticks per ms: {}", ticks_per_ms);

    let mut last_ticks = dwt.cyccnt.read();
    let mut accumulator: u32 = 0;
    let mut time_ms: u32 = 0;

    let mut initial_state = button_start.probe();
    let mut timer_start_ms: Option<u32> = None;

    let mut throttle = MIN_THROTTLE;
    let mut last_ui_update_ms = 0;
    let mut last_ui_render_ms = 0;
    let mut ramp_up_ms = 0;
    let mut offset_done = false;
    let mut weight = 0.0;
    let mut last_weight_ms = 0;
    let mut setpoint_reached = false;

    loop {
        let current_ticks = dwt.cyccnt.read();
        let delta = current_ticks.wrapping_sub(last_ticks);
        last_ticks = current_ticks;

        accumulator += delta;
        while accumulator >= ticks_per_ms {
            accumulator -= ticks_per_ms;
            time_ms += 1;
        }
        let button_start_state = button_start.probe();

        let (button_down_pulses, button_right_pulses, button_left_pulses) =
            cortex_m::interrupt::free(|cs| {
                if let Some(btns) = G_BUTTONS.borrow(cs).borrow_mut().as_mut() {
                    let d = btns.down.get_and_reset_pulses();
                    let r = btns.right.get_and_reset_pulses();
                    let l = btns.left.get_and_reset_pulses();
                    (d, r, l)
                } else {
                    (0, 0, 0)
                }
            });

        voltage_sensor.sample(time_ms);
        current_sensor.sample(time_ms);

        if button_down_pulses > 0 {
            ui.down();
        } else if button_right_pulses > 0 {
            ui.right();
        } else if button_left_pulses > 0 {
            ui.left();
        }

        if initial_state != button_start_state && setpoint_reached {
            if ui.timer_sec > 0.0 {
                if let Some(start_time) = timer_start_ms {
                    if time_ms - start_time >= (ui.timer_sec * 1000.0) as u32 {
                        initial_state = button_start_state;
                        timer_start_ms = None;
                    }
                } else {
                    timer_start_ms = Some(time_ms);
                }
            }
        }

        if time_ms - last_ui_render_ms >= 15 {
            last_ui_render_ms = time_ms;
            let starttime = dwt.cyccnt.read();
            ui.render_partial();
            let render_delta = dwt.cyccnt.read().wrapping_sub(starttime);
            if render_delta > 5000 {
                rprintln!("UI render time: {}", render_delta);
            }
        }

        if time_ms - ramp_up_ms >= 50 {
            ramp_up_ms = time_ms;
            if voltage_sensor.is_low() || voltage_sensor.is_unplugged() {
                initial_state = button_start_state;
            }

            if initial_state == button_start_state {
                timer_start_ms = None;
                offset_done = false;
                if throttle > MIN_THROTTLE {
                    throttle -= if throttle < 25.0 { 0.6 } else { 3.0 };
                }
                setpoint_reached = false;
            } else {
                if !offset_done {
                    let _ = weight_sensor.set_offset();
                    offset_done = true;
                }
                if ui.setpoint == Setpoint::Throttle {
                    if throttle < ui.throttle_setpoint {
                        throttle += if throttle < 25.0 { 0.4 } else { 3.0 };
                    } else {
                        setpoint_reached = true;
                    }
                } else if ui.setpoint == Setpoint::Current {
                    let current = current_sensor.get_current_abs();
                    if current < ui.current_setpoint - 0.5 {
                        throttle += 0.4
                    } else if current < ui.current_setpoint - 0.2 {
                        throttle += 0.1
                    } else if current < ui.current_setpoint - 0.01 {
                        setpoint_reached = true;
                        throttle += 0.03
                    } else if current > ui.current_setpoint + 0.01 {
                        setpoint_reached = true;
                        throttle -= 0.01
                    } else if current > ui.current_setpoint + 0.2 {
                        setpoint_reached = true;
                        throttle -= 0.1
                    } else if current > ui.current_setpoint + 0.5 {
                        setpoint_reached = true;
                        throttle -= 0.4
                    } else {
                        setpoint_reached = true;
                    }
                } else if ui.setpoint == Setpoint::Thrust {
                    let force_grams = weight;
                    let setpoint_grams = ui.thrust_setpoint / ui.force_unit_factor();
                    if force_grams < setpoint_grams - 20.0 {
                        throttle += 0.4
                    } else if force_grams < setpoint_grams - 3.0 {
                        throttle += 0.1
                    } else if force_grams < setpoint_grams - 0.3 {
                        setpoint_reached = true;
                        throttle += 0.03
                    } else if force_grams > setpoint_grams + 0.3 {
                        setpoint_reached = true;
                        throttle -= 0.01
                    } else if force_grams > setpoint_grams + 3.0 {
                        setpoint_reached = true;
                        throttle -= 0.1
                    } else if force_grams > setpoint_grams + 20.0 {
                        setpoint_reached = true;
                        throttle -= 0.4
                    } else {
                        setpoint_reached = true;
                    }
                }
            }
            if throttle > ui.throttle_limit {
                throttle = ui.throttle_limit;
            }

            cortex_m::interrupt::free(|cs| {
                *G_THROTTLE.borrow(cs).borrow_mut() = throttle;
            });

            if setpoint_reached {
                ui.engine_on();
            } else if throttle > MIN_THROTTLE {
                ui.engine_transition();
            } else {
                ui.engine_off();
            }
        }

        if time_ms - last_weight_ms >= 200 && initial_state != button_start_state {
            last_weight_ms = time_ms;
            weight = weight_sensor.get_weight().unwrap_or(0.0);
        }

        if time_ms - last_ui_update_ms
            >= if ui.displayed_ui == DisplayedUi::SensorReadings {
                1000
            } else {
                100
            }
        {
            last_ui_update_ms = time_ms;

            let time_left = timer_start_ms.map(|start_time| {
                let elapsed = (time_ms - start_time) as f32 / 1000.0;
                (ui.timer_sec - elapsed).max(0.0)
            });
            let weight_val = weight;
            let current_val = current_sensor.get_current_abs();
            let voltage_val = voltage_sensor.read();
            let v_per_cell = voltage_sensor.read_per_cell();

            ui.update(
                weight_val,
                current_val,
                voltage_val,
                v_per_cell,
                time_left,
                throttle,
            );
        }

        if delta > CLOCK_CYCLES_PER_SECOND / 100 {
            rprintln!("Delta: {}", delta);
        }
    }
}

#[interrupt]
fn TIM3() {
    static mut LOCAL_ESC: Option<EscController> = None;
    static mut TIME_MS: u32 = 0;

    // Fetch ESC on first run
    if LOCAL_ESC.is_none() {
        cortex_m::interrupt::free(|cs| {
            if let Some(esc) = G_ESC.borrow(cs).replace(None) {
                *LOCAL_ESC = Some(esc);
            }
        });
    }

    // Increment time (TIM3 is ~143Hz, so ~7ms per interrupt)
    *TIME_MS = TIME_MS.wrapping_add(7);
    let now_ms = *TIME_MS;

    // Handle ESC
    if let Some(esc) = LOCAL_ESC.as_mut() {
        // Read the current throttle calculated by the main loop
        let throttle = cortex_m::interrupt::free(|cs| *G_THROTTLE.borrow(cs).borrow());

        if throttle <= MIN_THROTTLE {
            esc.send_throttle(0.0, true);
        } else {
            esc.send_throttle(throttle, true);
        }
    }

    // Handle Buttons (Keep them in G_BUTTONS so main can access them)
    cortex_m::interrupt::free(|cs| {
        if let Some(btns) = G_BUTTONS.borrow(cs).borrow_mut().as_mut() {
            btns.down.update(now_ms);
            btns.right.update(now_ms);
            btns.left.update(now_ms);
        }
    });

    // Clear interrupt flag directly via registers
    unsafe {
        (*pac::TIM3::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}
