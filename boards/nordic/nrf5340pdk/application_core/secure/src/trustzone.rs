#![allow(dead_code)]

use enum_primitive::cast::FromPrimitive;
use kernel::debug;
use nrf53::app_peripheral_ids as app_id;
use nrf53::spu;

// Helper function that prints out the PC, useful for determining if code is within a secure
// region.
#[inline(always)]
pub unsafe fn print_pc() {
    let pc: u32;
    asm!("mov {}, pc", out(reg) pc);
    debug!("PC: {:#x}", pc);
}

extern "C" {
    static _ssecure_flash: u8;
    static _esecure_flash: u8;
    //static _snsc: u8;
    static SEC_FLASH_REG_SIZE: u8;
    static SEC_FLASH_NSC_SIZE: u8;
}

// Secure and Nonsecure region permissions.
static SEC_PERMS: u32 = spu::PERM_SECURE | spu::PERM_READ | spu::PERM_WRITE | spu::PERM_EXECUTE;
static NS_PERMS: u32 = spu::PERM_READ | spu::PERM_WRITE | spu::PERM_EXECUTE;

#[no_mangle]
#[inline(never)]
/// Set up TrustZone/SPU, allocating secure and non-secure regions.
///
/// Mark as CMSE nonsecure entry to clear registers before making BXNE back to
/// code that is newly marked as non-secure. Don't generate a trampoline because
/// this won't be called from a non-secure context (only called once before non-
/// secure contexts are allocated).
pub unsafe extern "C" fn setup(sau: &cortexm33::sau::SAU, spu: &nrf53::spu::Spu) {
    // must disable SAU and set the default mode to NS before the SPU can be used
    sau.disable_sau(cortexm33::sau::DefaultMode::NonSecure);

    let success = spu.init();
    assert!(success);

    // Get address and size of secure region from linker symbols.
    let sec_start = (&_ssecure_flash as *const u8) as usize;
    let sec_end = (&_esecure_flash as *const u8) as usize;
    let sec_reg_size = (&SEC_FLASH_REG_SIZE as *const u8) as usize;
    //let nsc_addr = (&_snsc as *const u8) as usize;
    //let nsc_size = (&SEC_FLASH_NSC_SIZE as *const u8) as usize;

    // set the GPIO as ns accessible
    let result = spu.set_periph_sec(nrf53::app_peripheral_ids::GPIO, false);
    assert!(result.is_ok());

    // set all gpio as ns accessible for now...
    for i in 0..nrf53::gpio::NUM_PINS {
        spu.set_gpio_sec(nrf53::gpio::Pin::from_usize(i).unwrap(), false);
    }

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

        let result = spu.set_flash_region_perms(i, perms);
        assert!(result.is_ok());
        assert!(spu.get_flash_region_perms(i).unwrap() == perms);
    }

    // add the nsc region
    // currently it is just defined within the linker script as 0x200 size
    // in the last secure region
    //let result = spu.add_nsc_region(spu::RegionType::Flash, nsc_addr / sec_reg_size, nsc_size);
    //assert!(result.is_ok());

    // TODO: use tt instr to check attrs
    //asm!("tt {}, {}", out(reg) output, in(reg) main_addr);
    //assert!(output & 1 << 10 == 0);

    /*let verify_addr = (verify as *const u8) as usize;
    let verify_region = verify_addr / sec_reg_size;
    assert!(SPU.get_flash_region_perms(verify_region).unwrap() == SEC_PERMS);*/
    // Configure corresponding secure region in RAM.
    /*for i in 0..spu::NUM_RAM_REGIONS {
        // Secure RAM regions are 8kib
        let reg_addr = i * (sec_reg_size / 2);
        // TODO: this is just wrong...
        //       this is testing if RAM addr is within secure flash bounds...
        //       which should never happend....

        /*let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            SEC_PERMS
        } else {
            NS_PERMS
        };*/
        let perms = NS_PERMS;

        let result = spu.set_ram_region_perms(i, perms);
        //assert!(result.is_ok());
        //assert!(SPU.get_ram_region_perms(i).unwrap() == perms);
    }*/

    spu.commit();
}

// TODO: rewrite verify function using tt instruction
/*#[inline(never)]
//#[cmse_nonsecure_entry]
/// Verify that TrustZone was setup properly.
pub unsafe extern "C" fn verify() -> bool {
    let mut success = true;
    let sec_start = (&_ssecure as *const u8) as usize;
    let sec_end = (&_esecure as *const u8) as usize;
    let sec_reg_size = (&SEC_FLASH_REG_SIZE as *const u8) as usize;

    //print_pc();

    // Verify flash region permissions.
    for i in 0..spu::NUM_FLASH_REGIONS {
        let reg_addr = i * sec_reg_size;
        let perms = if reg_addr >= sec_start && reg_addr < sec_end {
            // Print secure region numbers.
            //debug!("SEC{}", i);
            SEC_PERMS
        } else {
            NS_PERMS
        };

        let actual_perms = SPU.get_flash_region_perms(i).unwrap();
        if actual_perms != perms {
            // Flash Region has incorrect permissions, print out expected and actual values.
            //debug!("FR{}: ex {:#b}, got {:#b}", i, perms, actual_perms);
            success = false;
        }
    }

    // Verify RAM region permissions.
    /*for i in 0..spu::NUM_RAM_REGIONS {
        let reg_addr = i * (sec_reg_size / 2);
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
    }*/

    // Verify peripheral permissions.
    /*let gpio_perms = SPU.get_periph_sec(app_id::GPIO);
    debug!("GPIO: {:?}", gpio_perms.unwrap());
    let led_sec = SPU.get_gpio_sec(nrf53::gpio::Pin::P0_28);
    debug!("LED 1 sec: {:?}", led_sec);*/

    success
}*/
