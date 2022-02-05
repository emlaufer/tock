
#[link_section = ".nsc"]
#[naked]
#[no_mangle]
pub unsafe extern "C" fn test_gate_veneer() {
    asm!(
        "sg",
        "b.w {}",
        sym test_gate
    );
}

#[no_mangle]
fn test_gate() {
    unsafe {
        core::ptr::write_volatile(0x50842518 as *mut u32, 1 << 30);
        core::ptr::write_volatile(0x5084250C as *mut u32, 1 << 30);
        asm!("udf 100");
    }

    loop {}
}
