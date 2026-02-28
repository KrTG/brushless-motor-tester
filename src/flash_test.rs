#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32f7xx_hal::{flash::Flash, pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Flash test starting...");

    let dp = pac::Peripherals::take().unwrap();

    // We need some clocks for the delay, but flash doesn't strictly need them to be high
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let mut flash = Flash::new(dp.FLASH);

    // Sector 7 is usually safe for testing as it's far from small codebases
    // On F767 with 2MB, Sector 7 starts at 0x080C 0000 (offset 0xC0000)
    let sector = 7;
    let offset = 0xC_0000;
    let flash_addr = 0x080C0000 as *const u32;

    rprintln!("Reading initial value at 0x{:08X}...", flash_addr as usize);
    let initial_val = unsafe { core::ptr::read_volatile(flash_addr) };
    rprintln!("Initial value: 0x{:08X}", initial_val);

    rprintln!("Unlocking flash...");
    flash.unlock();

    rprintln!("Erasing sector {}...", sector);
    match flash.blocking_erase_sector(sector) {
        Ok(_) => rprintln!("Erase successful"),
        Err(e) => rprintln!("Erase failed: {:?}", e),
    }

    let test_data: [u8; 8] = [0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE];
    rprintln!("Programming data: {:02X?}", test_data);

    match flash.blocking_program(offset, &test_data) {
        Ok(_) => rprintln!("Program successful"),
        Err(e) => rprintln!("Program failed: {:?}", e),
    }

    rprintln!("Testing struct storage...");

    #[repr(C)]
    struct AppConfig {
        magic: u32,
        throttle_limit: u16,
        reserved: u16,
        name: [u8; 8],
    }

    let config = AppConfig {
        magic: 0x544F_4D52, // "RMOT"
        throttle_limit: 80,
        reserved: 0,
        name: *b"MOTOR001",
    };

    // Convert struct to byte slice
    let config_bytes = unsafe {
        core::slice::from_raw_parts(
            &config as *const AppConfig as *const u8,
            core::mem::size_of::<AppConfig>(),
        )
    };

    rprintln!("Unlocking flash for struct...");
    flash.unlock();

    // Using Sector 11 (the last one) for the struct test
    let struct_sector = 11;
    let struct_offset = 0x1C_0000;

    rprintln!("Erasing sector {}...", struct_sector);
    flash.blocking_erase_sector(struct_sector).unwrap();

    rprintln!("Programming struct data...");
    flash.blocking_program(struct_offset, config_bytes).unwrap();

    rprintln!("Locking flash...");
    flash.lock();

    rprintln!("Loading back from flash...");
    let loaded_config = unsafe { &*((0x08000000 + struct_offset) as *const AppConfig) };

    rprintln!("Loaded - Magic: 0x{:08X}", loaded_config.magic);
    rprintln!("Loaded - Throttle Limit: {}", loaded_config.throttle_limit);
    rprintln!(
        "Loaded - Name: {:?}",
        core::str::from_utf8(&loaded_config.name).unwrap_or("ERR")
    );

    rprintln!("Flash test complete.");

    loop {
        cortex_m::asm::nop();
    }
}
