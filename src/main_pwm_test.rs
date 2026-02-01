#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("PWM ESC Test (Hardware Validation)");

    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Disable D-Cache
    cp.SCB.disable_dcache(&mut cp.CPUID);
    cp.SCB.enable_icache();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpio = dp.GPIOE.split();

    // Configure PE9: Pull-up, Low speed
    let mut pe9_gpio = gpio.pe9.into_push_pull_output();
    pe9_gpio.set_high();

    let _pe9 = pe9_gpio
        .into_alternate::<1>()
        .set_speed(stm32f7xx_hal::gpio::Speed::Low)
        .internal_pull_up(true);

    // Initialize PWM manually (50Hz, standard ESC PWM)
    // TIM1 clock = 216 MHz
    // Goal: 50Hz = 20ms period
    // Strategy: Prescaler to get 1MHz tick rate, then ARR = 20000 for 20ms

    let timer_clock = clocks.pclk2().raw() * 2; // APB2 timer clock with multiplier
    rprintln!("Timer clock: {} Hz", timer_clock);

    // Target tick frequency: 1 MHz (1μs per tick)
    let tick_freq = 1_000_000u32;
    let prescaler = (timer_clock / tick_freq) - 1;

    // At 1MHz tick: 20000 ticks = 20ms = 50Hz
    let arr = 20000u32 - 1;

    unsafe {
        // Enable TIM1 clock in RCC - CRITICAL: must do this before touching TIM1 registers!
        let rcc = &*pac::RCC::ptr();
        rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());

        // Small delay for clock to stabilize
        cortex_m::asm::delay(16);

        let tim1 = &*pac::TIM1::ptr();

        // Disable timer
        tim1.cr1.modify(|_, w| w.cen().clear_bit());

        // Set prescaler for 1MHz tick rate
        tim1.psc.write(|w| w.bits(prescaler));

        // Set auto-reload (20000 ticks = 20ms @ 1MHz)
        tim1.arr.write(|w| w.bits(arr as u32));

        // Configure CH1 as PWM mode 1
        tim1.ccmr1_output().modify(|_, w| {
            w.cc1s()
                .bits(0b00) // Output
                .oc1m()
                .bits(0b110) // PWM mode 1
                .oc1pe()
                .set_bit() // Preload enable
        });

        // Enable CH1 output
        tim1.ccer.modify(|_, w| w.cc1e().set_bit());

        // Enable main output
        tim1.bdtr.modify(|_, w| w.moe().set_bit());

        // Enable auto-reload preload
        tim1.cr1.modify(|_, w| w.arpe().set_bit());

        // Start at 0% (idle)
        tim1.ccr1().write(|w| w.bits(0));

        // Enable counter
        tim1.cr1.modify(|_, w| w.cen().set_bit());
    }

    rprintln!(
        "PWM initialized: PSC={}, ARR={} (1μs ticks, 20ms period)",
        prescaler,
        arr
    );

    // Wait 2 seconds
    cortex_m::asm::delay(216_000_000 * 2);

    rprintln!("Arming ESC (1000μs pulse)...");
    let min_duty = 1000u32; // 1000μs = 1000 ticks @ 1MHz

    for _ in 0..200 {
        unsafe {
            let tim1 = &*pac::TIM1::ptr();
            tim1.ccr1().write(|w| w.bits(min_duty));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }

    rprintln!("Ramping to 1500μs pulse (mid throttle)...");
    let target_duty = 1500u32; // 1500μs = 1500 ticks @ 1MHz

    for i in 0..100 {
        let duty = min_duty + ((target_duty - min_duty) * i / 100);
        unsafe {
            let tim1 = &*pac::TIM1::ptr();
            tim1.ccr1().write(|w| w.bits(duty));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }

    rprintln!("Holding 7.5%% duty (1500μs)...");
    rprintln!("If motor spins NOW, hardware/wiring is GOOD.");
    rprintln!("If not, check: ESC power, signal wire, ground, ESC settings.");

    loop {
        unsafe {
            let tim1 = &*pac::TIM1::ptr();
            tim1.ccr1().write(|w| w.bits(target_duty));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }
}
