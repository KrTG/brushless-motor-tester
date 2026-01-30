use dshot_frame::{BidirectionalDshot, ErpmTelemetry, Frame};

use stm32f7xx_hal::{
    gpio::{Alternate, gpioe::PE9},
    pac::{DMA2, TIM1},
    prelude::*,
    timer::{Channel, Timer},
};

/// Possible errors from ESC communication
#[derive(Debug, Clone, Copy)]
pub enum EscError {
    /// Telemetry CRC check failed
    InvalidCrc,
    /// Motor poles configuration error
    InvalidPoles,
}

/// Status of the DMA transmission
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaState {
    /// Transmission started or completed successfully
    Ready,
    /// Hardware is still busy with the previous frame
    Busy,
}

#[repr(C, align(4))]
struct DshotBuffer<const N: usize> {
    data: [u16; N],
}

/// EscController handles "Real DShot" using DMA to stream PWM duty cycles.
pub struct EscController {
    dma2: DMA2,
    _tim1: Timer<TIM1>,
    ccr1_address: u32,
    motor_poles: u8,
    last_erpm: u32,
    is_armed: bool,
    is_blocking: bool,
    max_duty: u16,
    frame_buffer: DshotBuffer<17>,
    rx_buffer: DshotBuffer<64>,
}

impl EscController {
    pub fn new(
        timer: TIM1,
        pin: PE9<Alternate<1>>,
        frequency: fugit::HertzU32,
        motor_poles: u8,
        clocks: &stm32f7xx_hal::rcc::Clocks,
        dma2: DMA2,
        is_blocking: bool,
    ) -> Result<Self, EscError> {
        if motor_poles == 0 || motor_poles % 2 != 0 {
            return Err(EscError::InvalidPoles);
        }

        // Enable DMA2 clock. We use the PAC directly because the HAL
        // usually constrains the RCC before this driver is initialized.
        unsafe {
            let rcc = &*stm32f7xx_hal::pac::RCC::ptr();
            rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit());
        }

        // Calculate the address of CCR1 before we consume the timer.
        // On STM32F7, the CCR1 register is usually accessed via a method in the PAC.
        let ccr1_address = timer.ccr1().as_ptr() as u32;

        // Configure the timer for PWM using the HAL
        let mut pwm = timer.pwm_hz(pin, frequency, clocks);
        pwm.enable(Channel::C1);
        let max_duty = pwm.get_max_duty();
        pwm.set_duty(Channel::C1, 0);

        // Release the timer from the HAL PWM wrapper.
        let tim_raw = pwm.release();

        // Ensure TIM1 is in PWM Mode 1 (110) for Channel 1
        unsafe {
            let tim_pac = &*TIM1::ptr();
            tim_pac.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
        }

        Ok(Self {
            dma2,
            _tim1: tim_raw,
            ccr1_address,
            motor_poles,
            last_erpm: 0,
            is_armed: false,
            is_blocking,
            max_duty,
            frame_buffer: DshotBuffer { data: [0; 17] },
            rx_buffer: DshotBuffer { data: [0; 64] },
        })
    }

    /// Set the armed state
    pub fn set_armed(&mut self, armed: bool) {
        self.is_armed = armed;
    }

    /// Helper for the Beep command (1-5)
    pub fn beep(&mut self, strength: u8) {
        let cmd = match strength {
            1 => 1,
            2 => 2,
            3 => 3,
            4 => 4,
            _ => 5,
        };
        self.send_command(cmd, false);
    }

    /// Helper to request ESC Info
    pub fn request_info(&mut self) {
        self.send_command(6, true);
    }

    /// Send a specific DShot command (0-47) using DMA
    pub fn send_command(&mut self, command: u16, request_telemetry: bool) -> DmaState {
        let frame = Frame::<BidirectionalDshot>::new(command.min(47), request_telemetry)
            .expect("Command value 0-47 should be valid");
        self.transmit_frame(frame, request_telemetry)
    }

    /// Sends a throttle command to the ESC using DMA.
    pub fn send_throttle(&mut self, throttle_pct: f32, request_telemetry: bool) -> DmaState {
        let frame = self.create_throttle_frame(throttle_pct, request_telemetry);
        self.transmit_frame(frame, request_telemetry)
    }

    /// Internal helper to transmit any DShot frame via DMA
    fn transmit_frame(&mut self, frame: Frame<BidirectionalDshot>, requested: bool) -> DmaState {
        let stream = &self.dma2.st[1];

        // Check if DMA is busy before starting
        if stream.cr.read().en().is_enabled() {
            let lisr = self.dma2.lisr.read();
            if lisr.teif1().bit_is_set() {
                panic!("DMA transfer error detected from previous frame!");
            }
            if !lisr.tcif1().bit_is_set() {
                return DmaState::Busy;
            }
        }

        // 1. Prepare Timer for Transmission
        unsafe {
            let tim1 = &*TIM1::ptr();
            tim1.dier.modify(|_, w| w.cc1de().disabled());
            tim1.ccmr1_output()
                .modify(|_, w| w.cc1s().bits(0b00).oc1m().bits(0b110));
            tim1.bdtr.modify(|_, w| w.moe().set_bit());
            tim1.ccer.modify(|_, w| w.cc1e().set_bit());
            // Ensure the counter is definitely running
            tim1.cr1.modify(|_, w| w.cen().set_bit());
        }

        // Fill the frame buffer with the duty cycles for the frame
        // and ensure memory writes are visible to DMA - Data Memory Barrier
        // each value represents a pulse duration in timer clock cycles
        // how long the line is kept high or low
        // this is handled by the dshot frame library
        self.frame_buffer.data = frame.duty_cycles(self.max_duty);

        // NOTE: On STM32F7, if the L1 Data Cache is enabled, you MUST either:
        // 1. Clean the cache here: cortex_m_7::cache::CleanDataCacheByAddress(...)
        // 2. Or place frame_buffer in a Non-Cacheable (MPU) memory region.
        cortex_m::asm::dmb();

        // Disable DMA stream to reconfigure
        stream.cr.modify(|_, w| w.en().disabled());
        while stream.cr.read().en().is_enabled() {}

        // Clear DMA flags
        // ctcif1: Clear Transfer Complete Flag
        // chtif1: Clear Half Transfer Flag
        // cteif1: Clear Transfer Error Flag
        self.dma2
            .lifcr
            .write(|w| w.ctcif1().set_bit().chtif1().set_bit().cteif1().set_bit());

        // Set peripheral address (TIM1 CCR1) and memory address
        // We're writing from frame_buffer to TIM1 CCR1
        // so - from Memory Address Register to Peripheral Address Register
        stream.par.write(|w| unsafe { w.bits(self.ccr1_address) });
        stream
            .m0ar
            .write(|w| unsafe { w.bits(self.frame_buffer.data.as_ptr() as u32) });

        // Set the frame size (16 pulses + 1 "Shut up" pulse (pause with line kept low)
        stream.ndtr.write(|w| unsafe { w.bits(17) });

        stream.cr.write(|w| {
            w.chsel()
                .bits(6) // Select channel 6 (TIM1_CH1)
                .dir()
                .memory_to_peripheral() // Direction: memory to peripheral (TIM1 CCR1)
                .minc()
                .incremented() // Source: Incremented: Incremented writes from a sequence of memory addresses
                .pinc()
                .fixed() // Destination: Fixed: Fixed writes to a single peripheral address (TIM1 CCR1)
                .msize()
                .bits16() // Size of the CCR1 register - 16 bits
                .psize()
                .bits16() // Size of the CCR1 register - 16 bits
                .en()
                .enabled() // Enable the DMA stream
        });

        // 3. Start Transmission (Sync Timer to DMA)
        unsafe {
            let tim1 = &*TIM1::ptr();
            // Clear any pending interrupt/DMA flags to ensure a fresh start
            tim1.sr.write(|w| w.cc1if().clear_bit().uif().clear_bit());
            // Force a software update event to load shadow registers and reset counter
            tim1.egr.write(|w| w.ug().set_bit());
            // Small delay to ensure the update event is processed (Crucial on 216MHz F7)
            cortex_m::asm::nop();
            // Finally enable the DMA request
            tim1.dier.modify(|_, w| w.cc1de().enabled());
        }

        if self.is_blocking {
            while self.dma2.lisr.read().tcif1().bit_is_clear() {
                if self.dma2.lisr.read().teif1().bit_is_set() {
                    panic!("DMA TX Error");
                }
            }

            // --- TELEMETRY CAPTURE (GCR Decoder) ---
            if requested {
                let rx_stream = &self.dma2.st[3]; // Use Stream 3 for RX

                // a. Switch to Input Capture (Both Edges)
                unsafe {
                    let tim1 = &*TIM1::ptr();
                    tim1.dier.modify(|_, w| w.cc1de().disabled());
                    tim1.ccmr1_input()
                        .modify(|_, w| w.cc1s().bits(0b01).ic1f().bits(0b0011));
                    tim1.ccer
                        .modify(|_, w| w.cc1e().set_bit().cc1p().set_bit().cc1np().set_bit());
                }

                // 2. Configure RX DMA
                rx_stream.cr.modify(|_, w| w.en().disabled());
                while rx_stream.cr.read().en().is_enabled() {}

                self.dma2
                    .lifcr
                    .write(|w| w.ctcif3().set_bit().chtif3().set_bit().cteif3().set_bit());

                rx_stream
                    .par
                    .write(|w| unsafe { w.bits(self.ccr1_address) });
                rx_stream
                    .m0ar
                    .write(|w| unsafe { w.bits(self.rx_buffer.data.as_ptr() as u32) });
                rx_stream.ndtr.write(|w| unsafe { w.bits(42) }); // 21 bits * 2 edges

                rx_stream.cr.write(|w| {
                    w.chsel()
                        .bits(6)
                        .dir()
                        .peripheral_to_memory()
                        .minc()
                        .incremented()
                        .pinc()
                        .fixed()
                        .msize()
                        .bits16()
                        .psize()
                        .bits16()
                        .en()
                        .enabled()
                });

                // 4. Trigger RX Capture
                unsafe {
                    let tim1 = &*TIM1::ptr();
                    tim1.sr.write(|w| w.cc1if().clear_bit()); // Clear CC1 flag
                    tim1.dier.modify(|_, w| w.cc1de().enabled());
                }

                // 5. Wait for RX completion (with a small timeout/limit)
                let mut timeout = 20000;
                while self.dma2.lisr.read().tcif3().bit_is_clear() && timeout > 0 {
                    timeout -= 1;
                }

                if timeout > 0 {
                    // Extract bits from pulses
                    let mut bits = [false; 21];
                    let threshold = self.max_duty / 2;
                    for i in 0..21 {
                        let falling = self.rx_buffer.data[i * 2];
                        let rising = self.rx_buffer.data[i * 2 + 1];
                        let width = rising.wrapping_sub(falling);
                        bits[i] = width > threshold;
                    }

                    let decoded_u16 = self.decode_gcr(&bits);
                    let _ = self.update_telemetry(decoded_u16);
                }

                // 7. Restore Output mode readiness for next frame
                unsafe {
                    let tim1 = &*TIM1::ptr();
                    tim1.dier.modify(|_, w| w.cc1de().disabled());
                }
            }
        }

        DmaState::Ready
    }

    fn decode_gcr(&self, bits: &[bool; 21]) -> u16 {
        fn gcr_5to4(val: u8) -> u16 {
            match val {
                0x19 => 0x0,
                0x1B => 0x1,
                0x12 => 0x2,
                0x13 => 0x3,
                0x1D => 0x4,
                0x15 => 0x5,
                0x16 => 0x6,
                0x17 => 0x7,
                0x1A => 0x8,
                0x09 => 0x9,
                0x0A => 0xA,
                0x0B => 0xB,
                0x1E => 0xC,
                0x0D => 0xD,
                0x0E => 0xE,
                0x0F => 0xF,
                _ => 0,
            }
        }

        let mut result: u16 = 0;
        for i in 0..4 {
            let mut gcr_val: u8 = 0;
            for j in 0..5 {
                if bits[1 + i * 5 + j] {
                    gcr_val |= 1 << (4 - j);
                }
            }
            result |= gcr_5to4(gcr_val) << (12 - i * 4);
        }
        result
    }

    fn create_throttle_frame(
        &self,
        throttle_pct: f32,
        request_telemetry: bool,
    ) -> Frame<BidirectionalDshot> {
        if !self.is_armed {
            return Frame::new(0, request_telemetry).unwrap();
        }
        let throttle_pct = throttle_pct.clamp(0.0, 100.0);
        let throttle_value = if throttle_pct <= 0.0 {
            0
        } else {
            let range = 2047 - 48;
            48 + ((throttle_pct / 100.0) * range as f32) as u16
        };
        Frame::new(throttle_value, request_telemetry).expect("Valid throttle")
    }

    pub fn update_telemetry(&mut self, raw_bits: u16) -> Result<u32, EscError> {
        match ErpmTelemetry::try_from_raw(raw_bits) {
            Some(telem) => {
                self.last_erpm = telem.erpm();
                Ok(self.last_erpm)
            }
            None => Err(EscError::InvalidCrc),
        }
    }

    pub fn get_rpm(&self) -> u32 {
        let pole_pairs = self.motor_poles as u32 / 2;
        if pole_pairs == 0 {
            return 0;
        }
        self.last_erpm / pole_pairs
    }
}
