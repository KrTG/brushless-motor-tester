use stm32f7xx_hal::{
    adc::Adc,
    pac::{ADC_COMMON, ADC1},
    rcc::{APB2, Clocks},
};

/// Initializes ADC1 internally, reads VREFINT calibration, and returns VDDA
/// along with the ADC1 register for re-use.
pub fn get_avdd(
    adc_reg: ADC1,
    adc_common: &ADC_COMMON,
    apb2: &mut APB2,
    clocks: &Clocks,
) -> (f32, Adc<ADC1>) {
    // Initialize ADC1 temporarily
    let mut adc = Adc::adc1(adc_reg, apb2, clocks, 12, false);

    let raw_vrefint = adc.read_vref(adc_common);
    let vrefint_cal = unsafe { core::ptr::read_volatile(0x1FF0_F44A as *const u16) };

    // VDDA = 3.3V * VREFINT_CAL / DATA_VREFINT
    let vdda = 3.3 * (vrefint_cal as f32) / (raw_vrefint as f32);

    (vdda, adc)
}
