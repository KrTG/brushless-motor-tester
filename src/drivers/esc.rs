use dshot_frame::{Command, Frame, NormalDshot};

use stm32f7xx_hal::{
    gpio::{Alternate, gpioe::PE9},
    pac::{DMA2, TIM1},
    prelude::*,
    timer::{Channel, Timer},
};

#[repr(C, align(32))]
struct DshotBuffer<const N: usize> {
    data: [u16; N],
}

/// EscController handles "Real DShot" using DMA to stream PWM duty cycles.
pub struct EscController {
    dma2: DMA2,
    _tim1: Timer<TIM1>,
    ccr1_address: u32,
    max_duty: u16,
    frame_buffer: DshotBuffer<17>, // 16 bits + 1 trailing zero
}

impl EscController {
    pub fn new(
        timer: TIM1,
        pin: PE9<Alternate<1>>,
        frequency: fugit::HertzU32,
        clocks: &stm32f7xx_hal::rcc::Clocks,
        dma2: DMA2,
    ) -> Self {
        // Enable DMA2 clock
        unsafe {
            let rcc = &*stm32f7xx_hal::pac::RCC::ptr();
            rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit());
        }

        // Calculate the address of CCR1 before we consume the timer.
        let ccr1_address = timer.ccr1().as_ptr() as u32;

        // Configure the timer for PWM using the HAL
        let mut pwm = timer.pwm_hz(pin, frequency, clocks);
        pwm.enable(Channel::C1);
        let max_duty = pwm.get_max_duty();
        pwm.set_duty(Channel::C1, 0);

        // Release the timer
        let tim_raw = pwm.release();

        // Ensure TIM1 is in PWM Mode 1 (110) for Channel 1 and Enable Preload (OC1PE)
        unsafe {
            let tim_pac = &*TIM1::ptr();
            tim_pac
                .ccmr1_output()
                .modify(|_, w| w.oc1m().bits(0b110).oc1pe().set_bit());
            tim_pac.cr1.modify(|_, w| w.arpe().set_bit());
        }

        Self {
            dma2,
            _tim1: tim_raw,
            ccr1_address,
            max_duty,
            frame_buffer: DshotBuffer { data: [0; 17] },
        }
    }

    /// Helper for the Beep command (1-5)
    pub fn beep(&mut self, strength: u8) {
        let cmd = match strength {
            1 => Command::Beep1,
            2 => Command::Beep2,
            3 => Command::Beep3,
            4 => Command::Beep4,
            _ => Command::Beep5,
        };
        self.send_command(cmd);
    }

    /// Explicitly send a motor stop command
    pub fn send_stop(&mut self) {
        self.send_command(Command::MotorStop);
    }

    /// Send a specific DShot command (0-47) using DMA
    pub fn send_command(&mut self, command: Command) {
        let frame = Frame::<NormalDshot>::command(command, false);
        self.transmit_frame(frame)
    }

    /// Sends a throttle command to the ESC using DMA.
    pub fn send_throttle(&mut self, throttle_pct: f32) {
        let frame = self.create_throttle_frame(throttle_pct);
        self.transmit_frame(frame)
    }

    /// Internal helper to transmit any DShot frame via DMA
    fn transmit_frame(&mut self, frame: Frame<NormalDshot>) {
        let tx_stream = &self.dma2.st[1]; // Use Stream 1 for TIM1_CH1

        // 1. Prepare Timer for Transmission
        unsafe {
            let tim1 = &*TIM1::ptr();
            // Stop everything first
            tim1.cr1.modify(|_, w| w.cen().clear_bit());
            tim1.dier.modify(|_, w| w.cc1de().disabled()); // CC1 DMA

            // Output Mode 1 + Preload Enable
            tim1.ccmr1_output()
                .modify(|_, w| w.cc1s().bits(0b00).oc1m().bits(0b110).oc1pe().set_bit());

            // MOE (Main Output Enable) + OSSR (Off-State Selection for Run mode)
            tim1.bdtr.modify(|_, w| w.moe().set_bit().ossr().set_bit());
            tim1.ccer.modify(|_, w| {
                w.cc1e()
                    .set_bit()
                    .cc1p()
                    .clear_bit()
                    .cc1ne()
                    .clear_bit()
                    .cc1np()
                    .clear_bit()
            });

            // Auto-Reload Preload Enable + Repetition Counter 0
            tim1.cr1.modify(|_, w| w.arpe().set_bit());
            tim1.rcr.write(|w| w.rep().bits(0));

            // Reset Counter and set CCR1 to 0 (Idle)
            tim1.cnt.write(|w| w.bits(0));
            tim1.ccr1().write(|w| w.bits(0));

            // Force Update (UG) to load CCR1=0 into Shadow Register
            tim1.egr.write(|w| w.ug().set_bit());
            tim1.sr.write(|w| w.uif().clear_bit());
        }

        // Fill buffer with DShot frame data (17 elements: 16 bits + trailing zero)
        // The library's `duty_cycles` method returns [u16; 17] with the last element being 0.
        // We can just copy it directly.
        self.frame_buffer.data = frame.duty_cycles(self.max_duty);

        // Clean Cache
        cortex_m::asm::dsb();
        cortex_m::asm::dmb();

        // 2. Configure TX DMA (Stream 1 for TIM1_CH1)
        tx_stream.cr.modify(|_, w| w.en().disabled());
        while tx_stream.cr.read().en().is_enabled() {}

        // Clear flags for Stream 1
        self.dma2
            .lifcr
            .write(|w| w.ctcif1().set_bit().chtif1().set_bit().cteif1().set_bit());

        // Full DMA transfer: Buffer[0] to Buffer[End]
        tx_stream
            .par
            .write(|w| unsafe { w.bits(self.ccr1_address) });
        tx_stream
            .m0ar
            .write(|w| unsafe { w.bits(self.frame_buffer.data.as_ptr() as u32) });

        tx_stream.ndtr.write(|w| unsafe { w.bits(17) });

        tx_stream.cr.write(|w| {
            unsafe {
                w.chsel()
                    .bits(6) // Channel 6 for TIM1_CH1 on Stream 1
                    .msize()
                    .bits(0b01) // 16-bit
                    .psize()
                    .bits(0b01) // 16-bit
                    .pl()
                    .bits(0b11) // Very High Priority
            }
            .minc()
            .incremented()
            .pinc()
            .fixed()
            .dir()
            .memory_to_peripheral()
            .en()
            .enabled()
        });

        // 3. Start Transmission (Sync Timer to DMA)
        unsafe {
            let tim1 = &*TIM1::ptr();
            tim1.sr.write(|w| w.uif().clear_bit());

            // Ensure Shadow is 0
            tim1.ccr1().write(|w| w.bits(0));
            tim1.egr.write(|w| w.ug().set_bit());
            tim1.sr.write(|w| w.uif().clear_bit());

            // Enable CC1 DMA request
            tim1.dier.modify(|_, w| w.cc1de().enabled());

            // Clear CC1 interrupt flag
            tim1.sr.write(|w| w.cc1if().clear_bit());

            // START
            tim1.cr1.modify(|_, w| w.cen().set_bit());
        }

        // Wait for DMA Complete
        while self.dma2.lisr.read().tcif1().bit_is_clear() {
            if self.dma2.lisr.read().teif1().bit_is_set() {
                panic!("DMA TX Error");
            }
        }

        // Cleanup
        unsafe {
            let tim1 = &*TIM1::ptr();
            tim1.dier.modify(|_, w| w.cc1de().disabled());
            tim1.cr1.modify(|_, w| w.cen().disabled());
            tim1.ccr1().write(|w| w.bits(0));
        }
    }

    fn create_throttle_frame(&self, throttle_pct: f32) -> Frame<NormalDshot> {
        let throttle_pct = throttle_pct.clamp(0.0, 100.0);

        // range is 0 to 1999 (library adds 48 to whatever we give it)
        let range = 1999;
        let throttle_value = ((throttle_pct / 100.0) * range as f32) as u16;

        Frame::<NormalDshot>::new(throttle_value, false).expect("Valid throttle")
    }

    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }
}
