#![allow(dead_code)]

use kernel::debug;
use nrf53::app_peripheral_ids as app_id;
use nrf53::spu::{self, SPU};

// Helper function that prints out the PC, useful for determining if code is within a secure
// region.
#[inline(always)]
pub unsafe fn print_pc() {
    let pc: u32;
    asm!("mov {}, pc", out(reg) pc);
    debug!("PC: {:#x}", pc);
}

extern "C" {
    static _ssecure: u8;
    static _esecure: u8;
    static _snsc: u8;
    static SEC_FLASH_REG_SIZE: u8;
    static SEC_FLASH_NSC_SIZE: u8;
}

// Secure and Nonsecure region permissions.
static SEC_PERMS: u32 = spu::PERM_SECURE | spu::PERM_READ | spu::PERM_EXECUTE;
static NS_PERMS: u32 = spu::PERM_READ | spu::PERM_WRITE | spu::PERM_EXECUTE;

#[inline(never)]
#[cmse_nonsecure_entry]
#[link_section = ".secure.text"]
/// Set up TrustZone/SPU, allocating secure and non-secure regions.
/// NOTE: doesn't work!
///
/// Mark as CMSE nonsecure entry to clear registers before making BXNE back to
/// code that is newly marked as non-secure. Don't generate a trampoline because
/// this won't be called from a non-secure context (only called once before non-
/// secure contexts are allocated).
pub unsafe extern "C" fn setup() {
    let success = SPU.init();
    assert!(success);

    // Get address and size of secure region from linker symbols.
    let sec_start = (&_ssecure as *const u8) as usize;
    let sec_end = (&_esecure as *const u8) as usize;
    let sec_reg_size = (&SEC_FLASH_REG_SIZE as *const u8) as usize;
    let _nsc_addr = (&_snsc as *const u8) as usize;
    let _nsc_size = (&SEC_FLASH_NSC_SIZE as *const u8) as usize;

    // Configure secure region in flash.
    // FIXME: incorrect flash regions, need to figure out how to make sure that secure code is placed
    // within secure region boundaries in the final image. May require breaking out secure code
    // into its own binary?
    for i in 0..spu::NUM_FLASH_REGIONS {
        let reg_addr = i * sec_reg_size;
        let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            SEC_PERMS
        } else {
            NS_PERMS
        };

        let result = SPU.set_flash_region_perms(i, perms);
        assert!(result.is_ok());
        assert!(SPU.get_flash_region_perms(i).unwrap() == perms);
    }

    // Configure corresponding secure region in RAM.
    for i in 0..spu::NUM_RAM_REGIONS {
        // Secure RAM regions are 1/4 the size of secure flash regions in early nRF5340-PDKs. Not
        // true for later versions!
        let reg_addr = i * (sec_reg_size / 4);
        let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            SEC_PERMS
        } else {
            NS_PERMS
        };

        let result = SPU.set_ram_region_perms(i, perms);
        assert!(result.is_ok());
        assert!(SPU.get_ram_region_perms(i).unwrap() == perms);
    }

    // FIXME: not enabling the NSC regions should cause the code to crash, but doesn't
    /*
    let result = SPU.add_nsc_region(
        spu::RegionType::Ram,
        nsc_addr / (sec_reg_size / 4),
        nsc_size,
    );
    assert!(result.is_ok());
    */

    // Make LED 1 non-secure.
    SPU.set_gpio_sec(nrf53::gpio::Pin::P0_28, false);

    SPU.commit();
}

#[inline(never)]
#[cmse_nonsecure_entry]
#[link_section = ".secure.text"]
/// Verify that TrustZone was setup properly.
pub unsafe extern "C" fn verify() -> bool {
    let mut success = true;
    let sec_start = (&_ssecure as *const u8) as usize;
    let sec_end = (&_esecure as *const u8) as usize;
    let sec_reg_size = (&SEC_FLASH_REG_SIZE as *const u8) as usize;

    print_pc();

    // Verify flash region permissions.
    for i in 0..spu::NUM_FLASH_REGIONS {
        let reg_addr = i * sec_reg_size;
        let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            // Print secure region numbers.
            debug!("SEC{}", i);
            SEC_PERMS
        } else {
            NS_PERMS
        };

        let actual_perms = SPU.get_flash_region_perms(i).unwrap();
        if actual_perms != perms {
            // Flash Region has incorrect permissions, print out expected and actual values.
            debug!("FR{}: ex {:#b}, got {:#b}", i, perms, actual_perms);
            success = false;
        }
    }

    // Verify RAM region permissions.
    for i in 0..spu::NUM_RAM_REGIONS {
        let reg_addr = i * (sec_reg_size / 4);
        let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            // Print secure region numbers.
            debug!("SEC{}", i);
            SEC_PERMS
        } else {
            NS_PERMS
        };

        let actual_perms = SPU.get_ram_region_perms(i).unwrap();
        if actual_perms != perms {
            // Ram Region has incorrect permissions, print out expected and actual values.
            debug!("RR{}: ex {:#b}, got {:#b}", i, perms, actual_perms);
            success = false;
        }
    }

    // Verify peripheral permissions.
    let gpio_perms = SPU.get_periph_sec(app_id::GPIO);
    debug!("GPIO: {:?}", gpio_perms.unwrap());
    let led_sec = SPU.get_gpio_sec(nrf53::gpio::Pin::P0_28);
    debug!("LED 1 sec: {:?}", led_sec);

    success
}

// Test NSC function that just adds 2 to a number.
extern "C" {
    pub fn plus_two(x: u32) -> u32;
}

// Secure gateway trampoline, this is what you should call from non-secure code.
global_asm!(
    r#"
.section .secure.nsc, "ax"
.global plus_two
plus_two:
    sg
    b.w _nsc_plus_two
"#
);

#[no_mangle]
#[inline(never)]
#[cmse_nonsecure_entry]
#[link_section = ".secure.text"]
// Actual function, don't call this directly.
pub unsafe extern "C" fn _nsc_plus_two(x: u32) -> u32 {
    x + 2
}
