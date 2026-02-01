#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::rtt_init_print;

use cortex_m_rt::entry;
use stm32f7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    // Configure PE9 as TIM1_CH1
    let gpio = dp.GPIOE.split();
    let _pe9 = gpio.pe9.into_alternate::<1>();

    // Timer config: 1MHz tick rate, 20ms period (50Hz PWM)
    let timer_clock = clocks.pclk2().raw() * 2;
    let prescaler = (timer_clock / 1_000_000) - 1;
    let arr = 20_000 - 1;

    unsafe {
        let rcc = &*pac::RCC::ptr();
        rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
        cortex_m::asm::delay(16);

        let tim1 = &*pac::TIM1::ptr();
        tim1.cr1.modify(|_, w| w.cen().clear_bit());
        tim1.psc.write(|w| w.bits(prescaler));
        tim1.arr.write(|w| w.bits(arr));
        tim1.ccmr1_output()
            .modify(|_, w| w.cc1s().bits(0b00).oc1m().bits(0b110).oc1pe().set_bit());
        tim1.ccer.modify(|_, w| w.cc1e().set_bit());
        tim1.bdtr.modify(|_, w| w.moe().set_bit());
        tim1.cr1.modify(|_, w| w.arpe().set_bit());
        tim1.ccr1().write(|w| w.bits(0));
        tim1.cr1.modify(|_, w| w.cen().set_bit());
    }

    // Arm ESC: 1000μs pulses for 4 seconds
    for _ in 0..200 {
        unsafe {
            (*pac::TIM1::ptr()).ccr1().write(|w| w.bits(1000));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }

    // Ramp to 1100μs
    for i in 0..100 {
        let duty = 1000 + (100 * i / 100);
        unsafe {
            (*pac::TIM1::ptr()).ccr1().write(|w| w.bits(duty));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }

    // Hold throttle
    loop {
        unsafe {
            (*pac::TIM1::ptr()).ccr1().write(|w| w.bits(1100));
        }
        cortex_m::asm::delay(216_000_000 / 50);
    }
}
