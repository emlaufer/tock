//! Shared implementations for ARM Cortex-M33 MCUs.

#![crate_name = "cortexm33"]
#![crate_type = "rlib"]
#![feature(asm, asm_sym, naked_functions)]
#![no_std]

pub mod mpu;
pub mod sau;

// Re-export the base generic cortex-m functions here as they are
// valid on cortex-m33.
pub use cortexm::support;

pub use cortexm::initialize_ram_jump_to_main;
pub use cortexm::nvic;
pub use cortexm::print_cortexm_state as print_cortexm33_state;
pub use cortexm::scb;
pub use cortexm::syscall;
pub use cortexm::systick;
pub use cortexm::unhandled_interrupt;

/// These constants are defined in the linker script.
extern "C" {
    static _estack: u32;
    static mut _sstack: u32;
    static mut _szero: u32;
    static mut _ezero: u32;
    static mut _etext: u32;
    static mut _srelocate: u32;
    static mut _erelocate: u32;
}

/// Provide a `switch_to_user` function with exactly that name for syscall.rs.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[no_mangle]
pub unsafe extern "C" fn switch_to_user(
    user_stack: *const usize,
    process_regs: &mut [usize; 8],
) -> *const usize {
    cortexm::switch_to_user_arm_v7m(user_stack, process_regs)
}

#[cfg(not(any(target_arch = "arm", target_os = "none")))]
pub unsafe extern "C" fn switch_to_user(
    _user_stack: *const u8,
    _process_regs: &mut [usize; 8],
) -> *const usize {
    unimplemented!()
}

/// The `systick_handler` is called when the systick interrupt occurs, signaling
/// that an application executed for longer than its timeslice. This interrupt
/// handler is no longer responsible for signaling to the kernel thread that an
/// interrupt has occurred, but is slightly more efficient than the
/// `generic_isr` handler on account of not needing to mark the interrupt as
/// pending.
#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
#[naked]
pub unsafe extern "C" fn systick_handler() {
    asm!(
        "
    // Set thread mode to privileged to switch back to kernel mode.
    mov r0, #0
    msr CONTROL, r0
    /* CONTROL writes must be followed by ISB */
    /* http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0321a/BIHFJCAC.html */
    isb

    // This will resume in the switch to user function where application state
    // is saved and the scheduler can choose what to do next.
    bx   LR
    ",
        options(noreturn)
    );
}

/// This is called after a `svc` instruction, both when switching to userspace
/// and when userspace makes a system call.
#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
#[naked]
pub unsafe extern "C" fn svc_handler() {
    // TODO: fix this
    asm!(
        "
    // First check to see which direction we are going in. If the link register
    // is something other than 0xfffffff9, then we are coming from an app which
    // has called a syscall.
    cmp lr, #0xfffffff9
    bne 100f // to_kernel

    // If we get here, then this is a context switch from the kernel to the
    // application. Set thread mode to unprivileged to run the application.
    mov r0, #1
    msr CONTROL, r0
    /* CONTROL writes must be followed by ISB */
    /* http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0321a/BIHFJCAC.html */
    isb

    // Switch to the app.
    bx lr

  100: // to_kernel
    // An application called a syscall. We mark this in the global variable
    // `SYSCALL_FIRED` which is stored in the syscall file.
    // `UserspaceKernelBoundary` will use this variable to decide why the app
    // stopped executing.
    ldr r0, =SYSCALL_FIRED
    mov r1, #1
    str r1, [r0, #0]

    // Set thread mode to privileged as we switch back to the kernel.
    mov r0, #0
    msr CONTROL, r0
    /* CONTROL writes must be followed by ISB */
    /* http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0321a/BIHFJCAC.html */
    isb

    bx lr",
        options(noreturn)
    );
}

/// All ISRs are caught by this handler. This must ensure the interrupt is
/// disabled (per Tock's interrupt model) and then as quickly as possible resume
/// the main thread (i.e. leave the interrupt context). The interrupt will be
/// marked as pending and handled when the scheduler checks if there are any
/// pending interrupts.
///
/// If the ISR is called while an app is running, this will switch control to
/// the kernel.
#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
#[naked]
pub unsafe extern "C" fn generic_isr() {
    asm!(
        "
    // Set thread mode to privileged to ensure we are executing as the kernel.
    // This may be redundant if the interrupt happened while the kernel code
    // was executing.
    mov r0, #0
    msr CONTROL, r0
    /* CONTROL writes must be followed by ISB */
    /* http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0321a/BIHFJCAC.html */
    isb

    // Now need to disable the interrupt that fired in the NVIC to ensure it
    // does not trigger again before the scheduler has a chance to handle it. We
    // do this here in assembly for performance.
    //
    // The general idea is:
    // 1. Get the index of the interrupt that occurred.
    // 2. Set the disable bit for that interrupt in the NVIC.

    // Find the ISR number (`index`) by looking at the low byte of the IPSR
    // registers.
    mrs r0, IPSR       // r0 = Interrupt Program Status Register (IPSR)
    and r0, #0xff      // r0 = r0 & 0xFF
    sub r0, #16        // ISRs start at 16, so subtract 16 to get zero-indexed.

    // Now disable that interrupt in the NVIC.
    // High level:
    //    r0 = index
    //    NVIC.ICER[r0 / 32] = 1 << (r0 & 31)
    //
    lsrs r2, r0, #5    // r2 = r0 / 32

    // r0 = 1 << (r0 & 31)
    movs r3, #1        // r3 = 1
    and r0, r0, #31    // r0 = r0 & 31
    lsl r0, r3, r0     // r0 = r3 << r0

    // Load the ICER register address.
    mov r3, #0xe180    // r3 = &NVIC.ICER
    movt r3, #0xe000

    // Here:
    // - `r2` is index / 32
    // - `r3` is &NVIC.ICER
    // - `r0` is 1 << (index & 31)
    //
    // So we just do:
    //
    //  `*(r3 + r2 * 4) = r0`
    //
    str r0, [r3, r2, lsl #2]

    /* The pending bit in ISPR might be reset by hardware for pulse interrupts
     * at this point. So set it here again so the interrupt does not get lost
     * in service_pending_interrupts()
     * */
    /* r3 = &NVIC.ISPR */
    mov r3, #0xe200
    movt r3, #0xe000
    /* Set pending bit */
    str r0, [r3, r2, lsl #2]

    // Now we can return from the interrupt context and resume what we were
    // doing. If an app was executing we will switch to the kernel so it can
    // choose whether to service the interrupt.
    bx lr
    ",
        options(noreturn)
    );
}

#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
#[naked]
pub unsafe extern "C" fn hard_fault_handler() {
    // First need to determine if this a kernel fault or a userspace fault, and store
    // the unmodified stack pointer. Place these values in registers, then call
    // a non-naked function, to allow for use of rust code alongside inline asm.
    // Because calling a function increases the stack pointer, we have to check for a kernel
    // stack overflow and adjust the stack pointer before we branch

    asm!(
        "mov    r1, 0     /* r1 = 0 */",
        "mov    r2, 0     /* r2 = 0 */",
        //"tst    lr, #4    /* bitwise AND link register to 0b100 */",
        //"itte   eq        /* if lr==4, run next two instructions, else, run 3rd instruction. */",
        //"mrseq  r0, msp   /* r0 = kernel stack pointer */",
        //"addeq  r1, 1     /* r1 = 1, kernel was executing */",
        //"mrsne  r0, psp   /* r0 = userland stack pointer */",

        "tst    lr, #4    /* bitwise AND link register to 0b100 */",
        "bne    300f      /* if psp stack, goto 300 */",

        // TODO: pass it is nonnsecure to kernel hanlder
        // if from main stack, then check if the exception is from secure or nonsecure world
        "add    r1, 1     /* r1 = 1, kernel was executing */",
        "tst    lr, #64   /* if S bit not set, then use nonsecure stack */",
        "itee    eq",
        "mrseq   r0, msp_ns",
        "mrsne  r0, msp",
        "addne  r2, 1     /* r2 = 1, secure mode fault */",
        "b      301f",
        "300:             /* if psp stack */",
        // TODO: this should change to PSP_NS once we move processes to nonsecure world
        "mrs    r0, psp   /* assume psp stack is only used in secure world */",

        "301:             /* endif */",
        // Need to determine if we had a stack overflow before we push anything
        // on to the stack. We check this by looking at the BusFault Status
        // Register's (BFSR) `LSPERR` and `STKERR` bits to see if the hardware
        // had any trouble stacking important registers to the stack during the
        // fault. If so, then we cannot use this stack while handling this fault
        // or we will trigger another fault.
        "ldr   r4, =0xE000ED29  /* SCB BFSR register address */",
        "ldrb  r4, [r4]         /* r3 = BFSR */",
        "tst   r4, #0x30        /* r3 = BFSR & 0b00110000; LSPERR & STKERR bits */",
        "ite   ne               /* check if the result of that bitwise AND was not 0 */",
        "movne r3, #1           /* BFSR & 0b00110000 != 0; r3 = 1 */",
        "moveq r3, #0           /* BFSR & 0b00110000 == 0; r3 = 0 */",
        "and r5, r1, r3         /* bitwise and r2 and r1, store in r5 */ ",
        "cmp  r5, #1            /*  update condition codes to reflect if r3 == 1 && r1 == 1 */",
        "itt  eq                /* if r5==1 run the next 2 instructions, else skip to branch */",
        // if true, The hardware couldn't use the stack, so we have no saved data and
        // we cannot use the kernel stack as is. We just want to report that
        // the kernel's stack overflowed, since that is essential for
        // debugging.
        //
        // To make room for a panic!() handler stack, we just re-use the
        // kernel's original stack. This should in theory leave the bottom
        // of the stack where the problem occurred untouched should one want
        // to further debug.
        "ldreq  r4, ={}       /* load _estack into r4 */",
        "moveq  sp, r4        /* Set the stack pointer to _estack */",
        // finally, branch to non-naked handler
        // per ARM calling convention, faulting stack is passed in r0, kernel_stack in r1,
        // and whether there was a stack overflow in r2
        "b {}", // branch to function
        "bx lr", // if continued function returns, we need to manually branch to link register
        sym _estack, sym hard_fault_handler_continued,
        options(noreturn), // asm block never returns, so no need to mark clobbers
    );
}

#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
/// Continue the hardfault handler. This function is not `#[naked]`, meaning we can mix
/// `asm!()` and Rust. We separate this logic to not have to write the entire fault
/// handler entirely in assembly.
unsafe extern "C" fn hard_fault_handler_continued(
    faulting_stack: *mut u32,
    kernel_stack: u32,
    secure_mode: u32,
    stack_overflow: u32,
) {
    if kernel_stack != 0 {
        if stack_overflow != 0 {
            // Panic to show the correct error.
            panic!("kernel stack overflow");
        } else {
            // Show the normal kernel hardfault message.
            kernel_hardfault(faulting_stack, secure_mode);
        }
    } else {
        // Hard fault occurred in an app, not the kernel. The app should be
        // marked as in an error state and handled by the kernel.
        asm!(
            // TODO: check this
            "
        /* Read the relevant SCB registers. */
        ldr r0, =SCB_REGISTERS  /* Global variable address */
        ldr r1, =0xE000ED14     /* SCB CCR register address */
        ldr r2, [r1, #0]        /* CCR */
        str r2, [r0, #0]
        ldr r2, [r1, #20]       /* CFSR */
        str r2, [r0, #4]
        ldr r2, [r1, #24]       /* HFSR */
        str r2, [r0, #8]
        ldr r2, [r1, #32]       /* MMFAR */
        str r2, [r0, #12]
        ldr r2, [r1, #36]       /* BFAR */
        str r2, [r0, #16]

        ldr r0, =APP_HARD_FAULT /* Global variable address */
        mov r1, #1              /* r1 = 1 */
        str r1, [r0, #0]        /* APP_HARD_FAULT = 1 */

        /* Set thread mode to privileged */
        mov r0, #0
        msr CONTROL, r0
        /* CONTROL writes must be followed by ISB */
        /* http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0321a/BIHFJCAC.html */
        isb",
            out("r1") _,
            out("r0") _,
            out("r2") _,
            options(nostack),
        );
    }
}

#[cfg(all(
    target_arch = "arm",
    target_feature = "thumb-mode",
    target_os = "none"
))]
#[inline(never)]
unsafe fn kernel_hardfault(faulting_stack: *mut u32, secure_mode: u32) -> ! {
    let stacked_r0: u32 = *faulting_stack.offset(0);
    let stacked_r1: u32 = *faulting_stack.offset(1);
    let stacked_r2: u32 = *faulting_stack.offset(2);
    let stacked_r3: u32 = *faulting_stack.offset(3);
    let stacked_r12: u32 = *faulting_stack.offset(4);
    let stacked_lr: u32 = *faulting_stack.offset(5);
    let stacked_pc: u32 = *faulting_stack.offset(6);
    let stacked_xpsr: u32 = *faulting_stack.offset(7);

    let mode_str = if secure_mode != 0 {
        "Kernel (Secure)"
    } else {
        "Kernel (Non-secure)"
    };

    // if in nonsecure world, read the non-secure SCB registers
    let scb_offset: usize = if secure_mode != 0 {
        0x00000
    } else {
        // nonsecure SCB registers are aliased at a 0x20000 offset from the secure ones
        0x20000
    };

    let shcsr: u32 = core::ptr::read_volatile((0xE000ED24 + scb_offset) as *const u32);
    let cfsr: u32 = core::ptr::read_volatile((0xE000ED28 + scb_offset) as *const u32);
    let hfsr: u32 = core::ptr::read_volatile((0xE000ED2C + scb_offset) as *const u32);
    let mmfar: u32 = core::ptr::read_volatile((0xE000ED34 + scb_offset) as *const u32);
    let bfar: u32 = core::ptr::read_volatile((0xE000ED38 + scb_offset) as *const u32);
    let sfsr: u32 = core::ptr::read_volatile(0xE000EDE4 as *const u32);
    let sfar: u32 = core::ptr::read_volatile(0xE000EDE8 as *const u32);

    let iaccviol = (cfsr & 0x01) == 0x01;
    let daccviol = (cfsr & 0x02) == 0x02;
    let munstkerr = (cfsr & 0x08) == 0x08;
    let mstkerr = (cfsr & 0x10) == 0x10;
    let mlsperr = (cfsr & 0x20) == 0x20;
    let mmfarvalid = (cfsr & 0x80) == 0x80;

    let ibuserr = ((cfsr >> 8) & 0x01) == 0x01;
    let preciserr = ((cfsr >> 8) & 0x02) == 0x02;
    let impreciserr = ((cfsr >> 8) & 0x04) == 0x04;
    let unstkerr = ((cfsr >> 8) & 0x08) == 0x08;
    let stkerr = ((cfsr >> 8) & 0x10) == 0x10;
    let lsperr = ((cfsr >> 8) & 0x20) == 0x20;
    let bfarvalid = ((cfsr >> 8) & 0x80) == 0x80;

    let undefinstr = ((cfsr >> 16) & 0x01) == 0x01;
    let invstate = ((cfsr >> 16) & 0x02) == 0x02;
    let invpc = ((cfsr >> 16) & 0x04) == 0x04;
    let nocp = ((cfsr >> 16) & 0x08) == 0x08;
    let unaligned = ((cfsr >> 16) & 0x100) == 0x100;
    let divbysero = ((cfsr >> 16) & 0x200) == 0x200;

    let vecttbl = (hfsr & 0x02) == 0x02;
    let forced = (hfsr & 0x40000000) == 0x40000000;

    let invep = (sfsr & 0x01) == 0x01;
    let invis = (sfsr & 0x02) == 0x02;
    let inver = (sfsr & 0x04) == 0x04;
    let auviol = (sfsr & 0x08) == 0x08;
    let invtran = (sfsr & 0x10) == 0x10;
    let slsperr = (sfsr & 0x20) == 0x20;
    let sfarvalid = (sfsr & 0x40) == 0x40;
    let slserr = (sfsr & 0x80) == 0x80;

    let ici_it = (((stacked_xpsr >> 25) & 0x3) << 6) | ((stacked_xpsr >> 10) & 0x3f);
    let thumb_bit = ((stacked_xpsr >> 24) & 0x1) == 1;
    let exception_number = (stacked_xpsr & 0x1ff) as usize;

    panic!(
        "{} HardFault.\r\n\
         \tKernel version {}\r\n\
         \tr0  0x{:x}\r\n\
         \tr1  0x{:x}\r\n\
         \tr2  0x{:x}\r\n\
         \tr3  0x{:x}\r\n\
         \tr12 0x{:x}\r\n\
         \tlr  0x{:x}\r\n\
         \tpc  0x{:x}\r\n\
         \tprs 0x{:x} [ N {} Z {} C {} V {} Q {} GE {}{}{}{} ; ICI.IT {} T {} ; Exc {}-{} ]\r\n\
         \tsp  0x{:x}\r\n\
         \ttop of stack     0x{:x}\r\n\
         \tbottom of stack  0x{:x}\r\n\
         \tSHCSR 0x{:x}\r\n\
         \tCFSR  0x{:x}\r\n\
         \tHSFR  0x{:x}\r\n\
         \tInstruction Access Violation:       {}\r\n\
         \tData Access Violation:              {}\r\n\
         \tMemory Management Unstacking Fault: {}\r\n\
         \tMemory Management Stacking Fault:   {}\r\n\
         \tMemory Management Lazy FP Fault:    {}\r\n\
         \tInstruction Bus Error:              {}\r\n\
         \tPrecise Data Bus Error:             {}\r\n\
         \tImprecise Data Bus Error:           {}\r\n\
         \tBus Unstacking Fault:               {}\r\n\
         \tBus Stacking Fault:                 {}\r\n\
         \tBus Lazy FP Fault:                  {}\r\n\
         \tUndefined Instruction Usage Fault:  {}\r\n\
         \tInvalid State Usage Fault:          {}\r\n\
         \tInvalid PC Load Usage Fault:        {}\r\n\
         \tNo Coprocessor Usage Fault:         {}\r\n\
         \tUnaligned Access Usage Fault:       {}\r\n\
         \tDivide By Zero:                     {}\r\n\
         \tBus Fault on Vector Table Read:     {}\r\n\
         \tForced Hard Fault:                  {}\r\n\
         \tInvalid Secure Entry Point:         {}\r\n\
         \tInvalid Integrity Signature:        {}\r\n\
         \tInvalid Exception Return:           {}\r\n\
         \tAttribution Unit Violation:         {}\r\n\
         \tInvalid NS to S Transition:         {}\r\n\
         \tSecure Lazy FP Fault:               {}\r\n\
         \tSecure Lazy Activation Fault:       {}\r\n\
         \tFaulting Memory Address: (valid: {}) {:#010X}\r\n\
         \tBus Fault Address:       (valid: {}) {:#010X}\r\n\
         \tSecure Fault Address:    (valid: {}) {:#010X}\r\n\
         ",
        mode_str,
        option_env!("TOCK_KERNEL_VERSION").unwrap_or("unknown"),
        stacked_r0,
        stacked_r1,
        stacked_r2,
        stacked_r3,
        stacked_r12,
        stacked_lr,
        stacked_pc,
        stacked_xpsr,
        (stacked_xpsr >> 31) & 0x1,
        (stacked_xpsr >> 30) & 0x1,
        (stacked_xpsr >> 29) & 0x1,
        (stacked_xpsr >> 28) & 0x1,
        (stacked_xpsr >> 27) & 0x1,
        (stacked_xpsr >> 19) & 0x1,
        (stacked_xpsr >> 18) & 0x1,
        (stacked_xpsr >> 17) & 0x1,
        (stacked_xpsr >> 16) & 0x1,
        ici_it,
        thumb_bit,
        exception_number,
        ipsr_isr_number_to_str(exception_number),
        faulting_stack as u32,
        (_estack as *const ()) as u32,
        (&_sstack as *const u32) as u32,
        shcsr,
        cfsr,
        hfsr,
        iaccviol,
        daccviol,
        munstkerr,
        mstkerr,
        mlsperr,
        ibuserr,
        preciserr,
        impreciserr,
        unstkerr,
        stkerr,
        lsperr,
        undefinstr,
        invstate,
        invpc,
        nocp,
        unaligned,
        divbysero,
        vecttbl,
        forced,
        invep,
        invis,
        inver,
        auviol,
        invtran,
        slsperr,
        slserr,
        mmfarvalid,
        mmfar,
        bfarvalid,
        bfar,
        sfarvalid,
        sfar
    );
}

// Table 2-10 in ARM Cortex-M33 Devices Generic User Guide.
pub fn ipsr_isr_number_to_str(isr_number: usize) -> &'static str {
    match isr_number {
        0 => "Thread Mode",
        1 => "Reset",
        2 => "NMI",
        3 => "HardFault",
        4 => "MemManage",
        5 => "BusFault",
        6 => "UsageFault",
        7 => "SecureFault",
        8..=10 => "Reserved",
        11 => "SVCall",
        12 => "DebugMonitor",
        13 => "Reserved",
        14 => "PendSV",
        15 => "SysTick",
        16..=255 => "IRQn",
        _ => "(Unknown! Illegal value?)",
    }
}
