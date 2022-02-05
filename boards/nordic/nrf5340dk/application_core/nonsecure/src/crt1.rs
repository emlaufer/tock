// TODO: move these elsewhere....


//       Perhaps put all vectors in crt1, just with different section names
extern "C" {
    // _estack is not really a function, but it makes the types work
    // You should never actually invoke it!!
    fn _estack();
}

#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
/// ARM Cortex M Vector Table
pub static BASE_VECTORS: [unsafe extern "C" fn(); 16] = [
    // Stack Pointer
    _estack,
    // Reset Handler
    initialize_ram_jump_to_main,
    // NMI
    unhandled_interrupt,
    // Hard Fault
    unhandled_interrupt,
    // Memory Management Fault
    unhandled_interrupt,
    // Bus Fault
    unhandled_interrupt,
    // Usage Fault
    unhandled_interrupt,
    // Reserved
    unhandled_interrupt,
    // Reserved
    unhandled_interrupt,
    // Reserved
    unhandled_interrupt,
    // Reserved
    unhandled_interrupt,
    // SVCall
    unhandled_interrupt,
    // Reserved for Debug
    unhandled_interrupt,
    // Reserved
    unhandled_interrupt,
    // PendSv
    unhandled_interrupt,
    // SysTick
    unhandled_interrupt,
];


#[cfg_attr(
    all(target_arch = "arm", target_os = "none"),
    link_section = ".vectors"
)]
// used Ensures that the symbol is kept until the final binary
#[cfg_attr(all(target_arch = "arm", target_os = "none"), used)]
pub static IRQS: [unsafe extern "C" fn(); 80] = [unhandled_interrupt; 80];

#[cfg(all(target_arch = "arm", target_os = "none"))]
pub unsafe extern "C" fn unhandled_interrupt() {
    let mut interrupt_number: u32;


    // IPSR[8:0] holds the currently active interrupt
    asm!(
        "mrs r0, ipsr",
        out("r0") interrupt_number,
        options(nomem, nostack, preserves_flags)
    );

    interrupt_number = interrupt_number & 0x1ff;

    panic!("Unhandled Interrupt. ISR {} is active.", interrupt_number);
}

/// These constants are defined in the linker script.
extern "C" {
    static mut _sstack: u32;
    static mut _szero: u32;
    static mut _ezero: u32;
    static mut _etext: u32;
    static mut _srelocate: u32;
    static mut _erelocate: u32;
}

/// Assembly function to initialize the .bss and .data sections in RAM.
///
/// We need to (unfortunately) do these operations in assembly because it is
/// not valid to run Rust code without RAM initialized.
///
/// See https://github.com/tock/tock/issues/2222 for more information.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[naked]
pub unsafe extern "C" fn initialize_ram_jump_to_main() {
    asm!(
        "
    // Start by initializing .bss memory. The Tock linker script defines
    // `_szero` and `_ezero` to mark the .bss segment.
    ldr r0, ={sbss}     // r0 = first address of .bss
    ldr r1, ={ebss}     // r1 = first address after .bss

    movs r2, #0         // r2 = 0

  100: // bss_init_loop
    cmp r1, r0          // We increment r0. Check if we have reached r1
                        // (end of .bss), and stop if so.
    beq 101f            // If r0 == r1, we are done.
    stm r0!, {{r2}}     // Write a word to the address in r0, and increment r0.
                        // Since r2 contains zero, this will clear the memory
                        // pointed to by r0. Using `stm` (store multiple) with the
                        // bang allows us to also increment r0 automatically.
    b 100b              // Continue the loop.

  101: // bss_init_done

    // Now initialize .data memory. This involves coping the values right at the
    // end of the .text section (in flash) into the .data section (in RAM).
    ldr r0, ={sdata}    // r0 = first address of data section in RAM
    ldr r1, ={edata}    // r1 = first address after data section in RAM
    ldr r2, ={etext}    // r2 = address of stored data initial values

  200: // data_init_loop
    cmp r1, r0          // We increment r0. Check if we have reached the end
                        // of the data section, and if so we are done.
    beq 201f            // r0 == r1, and we have iterated through the .data section
    ldm r2!, {{r3}}     // r3 = *(r2), r2 += 1. Load the initial value into r3,
                        // and use the bang to increment r2.
    stm r0!, {{r3}}     // *(r0) = r3, r0 += 1. Store the value to memory, and
                        // increment r0.
    b 200b              // Continue the loop.

  201: // data_init_done

    // Now that memory has been initialized, we can jump to main() where the
    // board initialization takes place and Rust code starts.
    bl main
    ",
        sbss = sym _szero,
        ebss = sym _ezero,
        sdata = sym _srelocate,
        edata = sym _erelocate,
        etext = sym _etext,
        options(noreturn)
    );
}

