// M5Weight I2C Driver adapted from m5weight.cpp
// Focuses on direct translation of logic with simplified structure.

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub const DEVICE_DEFAULT_ADDR: u8 = 0x26;

// SCALES REGISTER
const WEIGHT_I2C_RAW_ADC_REG: u8 = 0x00;
const WEIGHT_I2C_CAL_DATA_REG: u8 = 0x10;
const WEIGHT_I2C_SET_GAP_REG: u8 = 0x40;
const WEIGHT_I2C_SET_OFFESET_REG: u8 = 0x50;
const WEIGHT_I2C_CAL_DATA_INT_REG: u8 = 0x60;
const WEIGHT_I2C_CAL_DATA_STRING_REG: u8 = 0x70;
const WEIGHT_I2C_FILTER_REG: u8 = 0x80;
// const JUMP_TO_BOOTLOADER_REG: u8 = 0xFD;
const FIRMWARE_VERSION_REG: u8 = 0xFE;
const I2C_ADDRESS_REG: u8 = 0xFF;

#[derive(Debug, Default)]
pub struct ForceSensorData {
    pub timestamp: u64,
    pub device_id: u8,
    pub lp_filter: u8,
    pub avg_filter: u8,
    pub ema_filter: u8,
    pub gap_value: f32,
    pub i2c_address: u8,
    pub firmware_version: u8,
    pub force: f32,
    pub force_int: i32,
    pub force_string: [u8; 16],
    pub raw_adc: i32,
}

pub struct M5Weight<I2C> {
    i2c: I2C,
    address: u8,
    pub disconnected: bool,
    pub lp_filter: u8,
    pub avg_filter: u8,
    pub ema_filter: u8,
    pub gap_value: f32,
    pub firmware_version: u8,
}

impl<I2C, E> M5Weight<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            disconnected: true,
            lp_filter: 0,
            avg_filter: 0,
            ema_filter: 0,
            gap_value: 0.0,
            firmware_version: 0,
        }
    }

    pub fn init(&mut self) -> Result<(), E> {
        // Equivalent to I2C::init() and setOffset()
        self.set_offset()
    }

    pub fn probe(&mut self) -> bool {
        let mut returned_address = [0u8; 1];
        if self
            .read_bytes(I2C_ADDRESS_REG, &mut returned_address)
            .is_ok()
        {
            return returned_address[0] == self.address;
        }
        false
    }

    fn read_bytes(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), E> {
        match self.i2c.write_read(self.address, &[reg], buffer) {
            Ok(_) => {
                self.disconnected = false;
                Ok(())
            }
            Err(e) => {
                self.disconnected = true;
                Err(e)
            }
        }
    }

    fn write_bytes(&mut self, reg: u8, buffer: &[u8]) -> Result<(), E> {
        // For no_std, we avoid vec!.
        // We need a buffer that can hold reg + data.
        // The max data length we write is 4 bytes (for float/int32).
        // So a small generic buffer on stack is sufficient.
        let mut data = [0u8; 16];

        if buffer.len() + 1 > data.len() {
            // In a robust driver we might panic or return a specific error,
            // but here we are constrained by the Error type E.
            // Given the usage is internal and valid, we assume it fits.
            // If really needed, we could do two writes if the device supported it,
            // but standard I2C write is usually one transaction.
            // For now, let's assume usage fits (max 4 bytes).
            // If we needed to support arbitrary length writes, we'd need a larger buffer or heap.
        }

        data[0] = reg;
        data[1..buffer.len() + 1].copy_from_slice(buffer);

        match self.i2c.write(self.address, &data[..buffer.len() + 1]) {
            Ok(_) => {
                self.disconnected = false;
                Ok(())
            }
            Err(e) => {
                self.disconnected = true;
                Err(e)
            }
        }
    }

    pub fn query_config(&mut self) {
        self.lp_filter = self.get_lp_filter().unwrap_or(0);
        self.avg_filter = self.get_avg_filter().unwrap_or(0);
        self.ema_filter = self.get_ema_filter().unwrap_or(0);
        self.gap_value = self.get_gap_value().unwrap_or(0.0);
        let addr = self.address;
        self.address = self.get_i2c_address().unwrap_or(addr);
        self.firmware_version = self.get_firmware_version().unwrap_or(0);
    }

    pub fn get_lp_filter(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.read_bytes(WEIGHT_I2C_FILTER_REG, &mut data)?;
        Ok(data[0])
    }

    pub fn set_lp_filter(&mut self, value: u8) -> Result<(), E> {
        self.write_bytes(WEIGHT_I2C_FILTER_REG, &[value])
    }

    pub fn get_avg_filter(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.read_bytes(WEIGHT_I2C_FILTER_REG + 1, &mut data)?;
        Ok(data[0])
    }

    pub fn set_avg_filter(&mut self, value: u8) -> Result<(), E> {
        self.write_bytes(WEIGHT_I2C_FILTER_REG + 1, &[value])
    }

    pub fn get_ema_filter(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.read_bytes(WEIGHT_I2C_FILTER_REG + 2, &mut data)?;
        Ok(data[0])
    }

    pub fn set_ema_filter(&mut self, value: u8) -> Result<(), E> {
        self.write_bytes(WEIGHT_I2C_FILTER_REG + 2, &[value])
    }

    pub fn get_weight(&mut self) -> Result<f32, E> {
        let mut data = [0u8; 4];
        self.read_bytes(WEIGHT_I2C_CAL_DATA_REG, &mut data)?;
        Ok(f32::from_le_bytes(data))
    }

    pub fn get_weight_int(&mut self) -> Result<i32, E> {
        let mut data = [0u8; 4];
        self.read_bytes(WEIGHT_I2C_CAL_DATA_INT_REG, &mut data)?;
        Ok(i32::from_le_bytes(data))
    }

    pub fn get_weight_string(&mut self) -> Result<[u8; 16], E> {
        let mut data = [0u8; 16];
        self.read_bytes(WEIGHT_I2C_CAL_DATA_STRING_REG, &mut data)?;
        Ok(data)
    }

    pub fn get_gap_value(&mut self) -> Result<f32, E> {
        let mut data = [0u8; 4];
        self.read_bytes(WEIGHT_I2C_SET_GAP_REG, &mut data)?;
        Ok(f32::from_le_bytes(data))
    }

    pub fn set_gap_value(&mut self, value: f32) -> Result<(), E> {
        self.write_bytes(WEIGHT_I2C_SET_GAP_REG, &value.to_le_bytes())
    }

    pub fn set_offset(&mut self) -> Result<(), E> {
        self.write_bytes(WEIGHT_I2C_SET_OFFESET_REG, &[1])
    }

    pub fn get_raw_adc(&mut self) -> Result<i32, E> {
        let mut data = [0u8; 4];
        self.read_bytes(WEIGHT_I2C_RAW_ADC_REG, &mut data)?;
        Ok(i32::from_le_bytes(data))
    }

    pub fn get_i2c_address(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.read_bytes(I2C_ADDRESS_REG, &mut data)?;
        Ok(data[0])
    }

    pub fn set_i2c_address(&mut self, address: u8) -> Result<(), E> {
        self.write_bytes(I2C_ADDRESS_REG, &[address])
    }

    pub fn get_firmware_version(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.read_bytes(FIRMWARE_VERSION_REG, &mut data)?;
        Ok(data[0])
    }

    /// Represents the Run() logic in the original C++ code
    pub fn run_step(&mut self) -> ForceSensorData {
        self.query_config();

        ForceSensorData {
            timestamp: 0, // Placeholder for hrt_absolute_time()
            device_id: self.address,
            lp_filter: self.lp_filter,
            avg_filter: self.avg_filter,
            ema_filter: self.ema_filter,
            gap_value: self.gap_value,
            i2c_address: self.address,
            firmware_version: self.firmware_version,
            force: self.get_weight().unwrap_or(0.0),
            force_int: self.get_weight_int().unwrap_or(0),
            force_string: self.get_weight_string().unwrap_or([0; 16]),
            raw_adc: self.get_raw_adc().unwrap_or(0),
        }
    }
}
