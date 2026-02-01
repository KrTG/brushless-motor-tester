use dshot_frame::{BidirectionalDshot, ErpmTelemetry, Frame};

use stm32f7xx_hal::{
    gpio::{Alternate, Output, PushPull, gpioe::PE9},
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
    last_raw_telemetry: u16,
    frame_buffer: DshotBuffer<20>, // Increased to 20 for padding (State High)
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
            last_raw_telemetry: 0,
            frame_buffer: DshotBuffer { data: [0; 20] },
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

    /// Get the last raw 16-bit telemetry frame (GCR decoded)
    pub fn get_last_telemetry(&self) -> u16 {
        self.last_raw_telemetry
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
        let tx_stream = &self.dma2.st[5]; // Use Stream 5 for TIM1_UP (Update Event)

        // Check if DMA is busy (Stream 5 is in HISR)
        if tx_stream.cr.read().en().is_enabled() {
            let hisr = self.dma2.hisr.read();
            if hisr.teif5().bit_is_set() {
                panic!("DMA TX error detected!");
            }
            if !hisr.tcif5().bit_is_set() {
                return DmaState::Busy;
            }
        }

        // 1. Prepare Timer for Transmission
        unsafe {
            let tim1 = &*TIM1::ptr();
            tim1.dier.modify(|_, w| w.ude().disabled()); // Disable Update DMA
            tim1.ccmr1_output()
                .modify(|_, w| w.cc1s().bits(0b00).oc1m().bits(0b110));
            tim1.bdtr.modify(|_, w| w.moe().set_bit());
            tim1.ccer.modify(|_, w| w.cc1e().set_bit());
            tim1.cr1.modify(|_, w| w.cen().set_bit());
        }

        // Fill the frame buffer with the duty cycles for the frame
        // and ensure memory writes are visible to DMA - Data Memory Barrier
        // each value represents a pulse duration in timer clock cycles
        // Fill buffer with 0 first (which means 0 duty cycle)
        // DShot frame is 16 bits. We have a buffer of 20.
        // We want the last few "pulses" to be 0 duty cycle, effectively holding the line low?
        // NO - DShot Idle is HIGH (100% duty, or just logic high).
        // Wait, DShot is digital.
        // 0 duty cycle = 0V.
        // The DShot frame ends, and the line should return to IDLE (Low? High?).
        // DShot Idle is LOW. (Unlike OneShot/PWM which idle Low).
        // Wait, check spec.
        // "The signal is low when idle."
        // So padding with 0 (0% duty cycle) is correct.

        let mut buffer = [0u16; 20];
        let duties = frame.duty_cycles(self.max_duty);
        for (i, d) in duties.iter().enumerate() {
            buffer[i] = *d;
        }
        // Remaining items in buffer are 0, which keeps line LOW.
        self.frame_buffer.data = buffer;

        // IMPORTANT for F7: If D-Cache is enabled, we MUST ensure the buffer is in RAM
        // and visible to DMA. Data Barrier is a start, but Cache Clean would be better.
        cortex_m::asm::dsb();
        cortex_m::asm::dmb();

        // 2. Configure TX DMA (Stream 5)
        tx_stream.cr.modify(|_, w| w.en().disabled());
        while tx_stream.cr.read().en().is_enabled() {}

        // Clear Stream 5 flags in HIFCR
        self.dma2
            .hifcr
            .write(|w| w.ctcif5().set_bit().chtif5().set_bit().cteif5().set_bit());

        // Set addresses for Stream 5
        // We will send the first bit manually and start DMA from the second bit
        tx_stream
            .par
            .write(|w| unsafe { w.bits(self.ccr1_address) });
        tx_stream
            .m0ar
            .write(|w| unsafe { w.bits(self.frame_buffer.data.as_ptr().offset(1) as u32) });

        // We send 17 bits now (16 data + 1 extra zero for safety/padding)
        tx_stream.ndtr.write(|w| unsafe { w.bits(17) });

        tx_stream.cr.write(|w| {
            unsafe {
                w.chsel()
                    .bits(6) // TIM1_UP is Channel 6 on Stream 5
                    .msize()
                    .bits(0b01) // 16-bit memory
                    .psize()
                    .bits(0b01) // 16-bit peripheral
            }
            .minc()
            .incremented() // Increment memory address
            .pinc()
            .fixed() // Fixed peripheral address (CCR1)
            .dir()
            .memory_to_peripheral() // Direction
            .en()
            .enabled()
        });

        // 3. Start Transmission (Sync Timer to DMA)
        unsafe {
            let tim1 = &*TIM1::ptr();
            // Clear any pending flags
            tim1.sr.write(|w| w.uif().clear_bit());
            // Reset the internal telemetry state
            self.last_raw_telemetry = 0;

            // CRITICAL: Load first bit manually into CCR1
            tim1.ccr1()
                .write(|w| w.bits(self.frame_buffer.data[0] as u32));

            // Enable Update DMA request and THEN trigger the update event
            // The UG event will latch the manual value into the shadow register
            // and trigger the DMA to load the NEXT value.
            tim1.dier.modify(|_, w| w.ude().enabled());
            tim1.egr.write(|w| w.ug().set_bit());
        }

        if self.is_blocking {
            while self.dma2.hisr.read().tcif5().bit_is_clear() {
                if self.dma2.hisr.read().teif5().bit_is_set() {
                    panic!("DMA TX Error");
                }
            }
            // Disable the Update DMA request once transfer is done
            unsafe {
                let tim1 = &*TIM1::ptr();
                tim1.dier.modify(|_, w| w.ude().disabled());
            }

            // --- TELEMETRY CAPTURE (GCR Decoder) ---
            if requested {
                let rx_stream = &self.dma2.st[3]; // Use Stream 3 for RX

                // a. Switch to Input Capture (Both Edges)
                unsafe {
                    let tim1 = &*TIM1::ptr();
                    // ic1f: Filter = 0 (No filter for fast DShot pulses)
                    tim1.ccmr1_input()
                        .modify(|_, w| w.cc1s().bits(0b01).ic1f().bits(0b0000));
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

                // 5. Wait for RX completion (With large timeout ~5ms)
                let mut timeout = 1_000_000;
                while self.dma2.lisr.read().tcif3().bit_is_clear() && timeout > 0 {
                    timeout -= 1;
                    cortex_m::asm::nop();
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
                    self.last_raw_telemetry = decoded_u16;
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

        // Critical Fix: 0.0% must map to Command 0 (Disarmed/Stop)
        // Anything > 0.0% maps to the throttle range [48, 2047]
        let throttle_value = if throttle_pct <= 0.0 {
            0 // Command 0
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

    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }
}
