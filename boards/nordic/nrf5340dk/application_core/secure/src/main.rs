//! Tock kernel for the Nordic Semiconductor nRF5340 preview development kit (PDK).
//!
//! It is based on nRF5340 SoC (dual Cortex M33 cores with a BLE transceiver) with
//! many exported I/O and peripherals.
//!
//! Pin Configuration
//! -------------------
//!
//! ### `GPIO`
//!
//! | #  | Pin   | Ix | Header | Arduino |
//! |----|-------|----|--------|---------|
//! | 0  | P1.01 | 33 | P3 1   | D0      |
//! | 1  | P1.02 | 34 | P3 2   | D1      |
//! | 2  | P1.03 | 35 | P3 3   | D2      |
//! | 3  | P1.04 | 36 | P3 4   | D3      |
//! | 4  | P1.05 | 37 | P3 5   | D4      |
//! | 5  | P1.06 | 38 | P3 6   | D5      |
//! | 6  | P1.07 | 39 | P3 7   | D6      |
//! | 7  | P1.08 | 40 | P3 8   | D7      |
//! | 8  | P1.10 | 42 | P4 1   | D8      |
//! | 9  | P1.11 | 43 | P4 2   | D9      |
//! | 10 | P1.12 | 44 | P4 3   | D10     |
//! | 11 | P1.13 | 45 | P4 4   | D11     |
//! | 12 | P1.14 | 46 | P4 5   | D12     |
//! | 13 | P1.15 | 47 | P4 6   | D13     |
//! | 14 | P0.26 | 26 | P4 9   | D14     |
//! | 15 | P0.27 | 27 | P4 10  | D15     |
//!
//! ### `GPIO` / Analog Inputs
//!
//! | #  | Pin        | Header | Arduino |
//! |----|------------|--------|---------|
//! |    | P0.04 AIN1 | P2 1   | A0      |
//! |    | P0.05 AIN2 | P2 2   | A1      |
//! |    | P0.06 AIN4 | P2 3   | A2      |
//! |    | P0.07 AIN5 | P2 4   | A3      |
//! |    | P0.25 AIN6 | P2 5   | A4      |
//! |    | P0.26 AIN7 | P2 6   | A5      |
//!
//! ### Onboard Functions
//!
//! | Pin   | Header | Function |
//! |-------|--------|----------|
//! | P0.08 | P6  6  | Button 3 |
//! | P0.09 | P6  7  | Button 4 |
//! | P0.13 | P24 3  | SPI MOSI |
//! | P0.14 | P24 4  | SPI MISO |
//! | P0.17 | P24 7  | SPI CLK  |
//! | P0.19 | P24 9  | UART RTS |
//! | P0.20 | P24 10 | UART TXD |
//! | P0.21 | P24 11 | UART CTS |
//! | P0.22 | P24 12 | UART RXD |
//! | P0.23 | P24 13 | Button 1 |
//! | P0.24 | P24 14 | Button 2 |
//! | P0.28 | P24 15 | LED 1    |
//! | P0.29 | P24 16 | LED 2    |
//! | P0.30 | P24 17 | LED 3    |
//! | P0.31 | P24 18 | LED 4    |

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![feature(asm, asm_sym)]
#![feature(cmse_nonsecure_entry)]
#![feature(abi_c_cmse_nonsecure_call)]
#![feature(global_asm)]
#![feature(naked_functions)]
//#![deny(missing_docs)]

use kernel::component::Component;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil::led::LedLow;
use kernel::hil::time::Counter;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
#[allow(unused_imports)]
use kernel::{capabilities, create_capability, debug, debug_flush_queue, debug_gpio, debug_verbose, static_init};
use nrf53::chip::Nrf53AppDefaultPeripherals;
use nrf53::gpio::Pin;


// The nRF5340PDK LEDs (see back of board)
const LED1_PIN: Pin = Pin::P0_28;
const LED2_PIN: Pin = Pin::P0_29;
const LED3_PIN: Pin = Pin::P0_30;
const LED4_PIN: Pin = Pin::P0_31;

// The nRF5340PDK buttons (see back of board)
const BUTTON1_PIN: Pin = Pin::P0_23;
const BUTTON2_PIN: Pin = Pin::P0_24;
const BUTTON3_PIN: Pin = Pin::P0_08;
const BUTTON4_PIN: Pin = Pin::P0_09;

const UART_RTS: Option<Pin> = Some(Pin::P0_19);
const UART_TXD: Pin = Pin::P0_20;
const UART_CTS: Option<Pin> = Some(Pin::P0_21);
const UART_RXD: Pin = Pin::P0_22;

#[allow(dead_code)]
const SPI_MOSI: Pin = Pin::P0_13;
#[allow(dead_code)]
const SPI_MISO: Pin = Pin::P0_14;
#[allow(dead_code)]
const SPI_CLK: Pin = Pin::P0_17;

#[allow(dead_code)]
const SPI_MX25R6435F_CHIP_SELECT: Pin = Pin::P0_18;
#[allow(dead_code)]
const SPI_MX25R6435F_WRITE_PROTECT_PIN: Pin = Pin::P0_15;
#[allow(dead_code)]
const SPI_MX25R6435F_HOLD_PIN: Pin = Pin::P0_16;

/// Debug Writer
pub mod io;

/// Secure mode gates and veneers
pub mod gates;

/// ARM TrustZone/SPU support.
mod trustzone;

/// Tests
#[allow(dead_code)]
mod tests;

// Whether to use UART debugging or Segger RTT (USB) debugging.
// - Set to false to use UART.
// - Set to true to use Segger RTT over USB.
const USB_DEBUGGING: bool = false;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 8;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

static mut CHIP: Option<&'static nrf53::chip::NRF53<Nrf53AppDefaultPeripherals>> = None;
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

/// Supported drivers by the platform
pub struct Platform {
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf53::rtc::Rtc<'static>>,
    >,
    button: &'static capsules::button::Button<'static, nrf53::gpio::GPIOPin<'static>>,
    pconsole: &'static capsules::process_console::ProcessConsole<
        'static,
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf53::rtc::Rtc<'static>>,
        components::process_console::Capability,
    >,
    console: &'static capsules::console::Console<'static>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf53::gpio::GPIOPin<'static>>,
    led: &'static capsules::led::LedDriver<
        'static,
        LedLow<'static, nrf53::gpio::GPIOPin<'static>>,
        4,
    >,
    ipc: kernel::ipc::IPC<NUM_PROCS>,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm33::systick::SysTick,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            //capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

impl KernelResources<nrf53::chip::NRF53<'static, Nrf53AppDefaultPeripherals<'static>>>
    for Platform
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm33::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        &self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

//impl KernelResources<nrf53::chip::NRF53<nrf53::

#[inline(never)]
unsafe fn get_peripherals() -> &'static mut Nrf53AppDefaultPeripherals<'static> {
    // Init chip peripheral drivers
    let nrf53_app_peripherals = static_init!(
        Nrf53AppDefaultPeripherals,
        Nrf53AppDefaultPeripherals::new()
    );

    nrf53_app_peripherals
}

// nothing for now...

// Sample call to secure function from non-secure code
/*#[link_section = ".secure.text"]
#[inline(never)]
fn test_nonsecure() {
    unsafe {
        core::ptr::write_volatile(0x40842518 as *mut u32, 1 << 29);
        core::ptr::write_volatile(0x4084250C as *mut u32, 1 << 29);

        loop {}
        // try calling a secure function
        //test_secure_gate();
    }
}*/

/*extern "C" {
    /// hi
    pub fn test_secure_gate();
}

// Secure gateway trampoline, this is what you should call from non-secure code.
global_asm!(
    r#"
.section .secure.nsc, "ax"
.global test_secure_gate
test_secure_gate:
    sg
    b.w test_secure_impl
"#
);

/// test sec
#[cmse_nonsecure_entry]
#[no_mangle]
#[inline(never)]
pub unsafe extern "C" fn test_secure_impl() {
    unsafe {
        core::ptr::write_volatile(0x50842518 as *mut u32, 1 << 30);
        core::ptr::write_volatile(0x5084250C as *mut u32, 1 << 30);
    }
}*/

extern "C" {
    static _ns_estack: u32;
    static _ns_sstack: u32;
}

/*#[link(name = "nrf5340pdk_app_nonsecure")]
extern "C" {
    fn test_nonsecure();
}*/

/*#[no_mangle]
#[inline(never)]
/// HI
pub fn jump_to_ns() {
    unsafe {
    asm!("ldr {tmp}, ={symbol}
          msr MSP_NS, {tmp}",
           tmp = out(reg) _, symbol = sym _ns_estack);
    let one = 1;
    asm!("bics {0}, {1}
          bxns {0}", in(reg) test_nonsecure, in(reg) one);
    }
    loop {}
}*/
macro_rules! led1 {
    () => {
        unsafe {
            core::ptr::write_volatile(0x50842518 as *mut u32, 1 << 28);
            core::ptr::write_volatile(0x5084250C as *mut u32, 1 << 28);
        }
    };
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    // Loads relocations and clears BSS
    nrf53::app_init();

    // TODO: trustzone first or second?
    let nrf53_app_peripherals = get_peripherals();

    // TODO: figure out why this isn't working
    // disable appprotect
    //core::ptr::write_volatile(0x00FF8000 as *mut u32, 0x50FA50FA);
    //core::ptr::write_volatile(0x00FF801C as *mut u32, 0x50FA50FA);
    //core::ptr::write_volatile(0x50006010 as *mut u32, 0x50FA50FA);
    //core::ptr::write_volatile(0x50006014 as *mut u32, 0x50FA50FA);
    // Setup trustzone, all following code will be executed in non-secure world.
    //panic!("Hi");
    //show_led(LED::LED1);

    // THIS SHOULD GENERATE SECUREITY FAULT if we call when in secure mode...
    //test_nonsecure();
    // must ensure bottom bit is 0...
    // TODO: Weird issue: cannot use this, because this won't actually move the test_nonsecure
    // function into the address ....
    /*let nonsecure_ptr = core::mem::transmute::<usize, extern "C-cmse-nonsecure-call" fn()>(test_nonsecure as usize);
    nonsecure_ptr();*/

    //if trustzone::verify() {}

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    let gpio = components::gpio::GpioComponent::new(
        board_kernel,
        capsules::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            nrf53::gpio::GPIOPin,
            0 => &nrf53_app_peripherals.gpio_port[Pin::P1_01],
            1 => &nrf53_app_peripherals.gpio_port[Pin::P1_04],
            2 => &nrf53_app_peripherals.gpio_port[Pin::P1_05],
            3 => &nrf53_app_peripherals.gpio_port[Pin::P1_06],
            4 => &nrf53_app_peripherals.gpio_port[Pin::P1_07],
            5 => &nrf53_app_peripherals.gpio_port[Pin::P1_08],
            6 => &nrf53_app_peripherals.gpio_port[Pin::P1_09],
            7 => &nrf53_app_peripherals.gpio_port[Pin::P1_10],
            8 => &nrf53_app_peripherals.gpio_port[Pin::P1_11],
            9 => &nrf53_app_peripherals.gpio_port[Pin::P1_12],
            10 => &nrf53_app_peripherals.gpio_port[Pin::P1_13],
            11 => &nrf53_app_peripherals.gpio_port[Pin::P1_14],
            12 => &nrf53_app_peripherals.gpio_port[Pin::P1_15]
        ),
    )
    .finalize(components::gpio_component_buf!(nrf53::gpio::GPIOPin));

    let button = components::button::ButtonComponent::new(
        board_kernel,
        capsules::button::DRIVER_NUM,
        components::button_component_helper!(
            nrf53::gpio::GPIOPin,
            (
                &nrf53_app_peripherals.gpio_port[BUTTON1_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            ), //23
            (
                &nrf53_app_peripherals.gpio_port[BUTTON2_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            ), //24
            (
                &nrf53_app_peripherals.gpio_port[BUTTON3_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            ), //8
            (
                &nrf53_app_peripherals.gpio_port[BUTTON4_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            ) //9
        ),
    )
    .finalize(components::button_component_buf!(nrf53::gpio::GPIOPin));

    let led = components::led::LedsComponent::new().finalize(components::led_component_helper!(
        LedLow<'static, nrf53::gpio::GPIOPin>,
        LedLow::new(&nrf53_app_peripherals.gpio_port[LED1_PIN]),
        LedLow::new(&nrf53_app_peripherals.gpio_port[LED2_PIN]),
        LedLow::new(&nrf53_app_peripherals.gpio_port[LED3_PIN]),
        LedLow::new(&nrf53_app_peripherals.gpio_port[LED4_PIN]),
    ));

    let chip = static_init!(
        nrf53::chip::NRF53<Nrf53AppDefaultPeripherals>,
        nrf53::chip::NRF53::new(nrf53_app_peripherals)
    );
    CHIP = Some(chip);

    // TODO: this should be moved to as early as possible
    //       because some peripheral addresses can be changed, and we need them to stay
    //       consistent for initialization. For example, the UARTE moves to a 0x4... address
    //       because we allow access in nonsecure mode
    //trustzone::setup(&chip.sau, &nrf53_app_peripherals.spu);

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let gpio_port = &nrf53_app_peripherals.gpio_port;
    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(Some(&gpio_port[LED1_PIN]), Some(&gpio_port[LED2_PIN]), None);

    let rtc = &nrf53_app_peripherals.rtc0;
    let _ = rtc.start();
    let mux_alarm = components::alarm::AlarmMuxComponent::new(rtc)
        .finalize(components::alarm_mux_component_helper!(nrf53::rtc::Rtc));
    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_helper!(nrf53::rtc::Rtc));

    // CALLER: should be in secure world.... called by 
    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 2], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    let uart_channel: &dyn kernel::hil::uart::Uart = if USB_DEBUGGING {
        // TODO: for some reason, initializeation of this causes showing led1 to panic
        // Initialize early so any panic beyond this point can use the RTT memory object.
        let mut rtt_memory_refs =
            components::segger_rtt::SeggerRttMemoryComponent::new().finalize(());

        // XXX: This is inherently unsafe as it aliases the mutable reference to rtt_memory. This
        // aliases reference is only used inside a panic handler, which should be OK, but maybe we
        // should use a const reference to rtt_memory and leverage interior mutability instead.
        self::io::set_rtt_memory(&mut *rtt_memory_refs.get_rtt_memory_ptr());

        let rtt = components::segger_rtt::SeggerRttComponent::new(mux_alarm, rtt_memory_refs)
            .finalize(components::segger_rtt_component_helper!(nrf53::rtc::Rtc));
        rtt
    } else {
        nrf53_app_peripherals.uarte0.initialize(
            nrf53::pinmux::Pinmux::new(UART_TXD as u32),
            nrf53::pinmux::Pinmux::new(UART_RXD as u32),
            UART_CTS.map(|x| nrf53::pinmux::Pinmux::new(x as u32)),
            UART_RTS.map(|x| nrf53::pinmux::Pinmux::new(x as u32)),
        );
        &nrf53_app_peripherals.uarte0
    };

    let process_printer =
        components::process_printer::ProcessPrinterTextComponent::new().finalize(());
    PROCESS_PRINTER = Some(process_printer);

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux =
        components::console::UartMuxComponent::new(uart_channel, 115200, dynamic_deferred_caller)
            .finalize(());

    let pconsole = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
    )
    .finalize(components::process_console_component_helper!(
        nrf53::rtc::Rtc<'static>
    ));

    //Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux).finalize(());

    ///// These symbols are defined in the linker script.
    //extern "C" {
    //    /// Beginning of the ROM region containing the non-secure kernel.
    //    static _sns_flash: u8;
    //    /// End of the ROM region containing the non-secure kernel.
    //    static _ens_flash: u8;
    //}

    //let ns_vector_table = core::slice::from_raw_parts((&_sns_flash as *const u8) as *const usize, 2);
    //let ns_estack = ns_vector_table[0];
    //let ns_start = ns_vector_table[1];

    //// TODO: fix debug writer...wont flush`
    ////panic!("Got addresses: {:x} {:x}", ns_estack, ns_start);
    ////loop {};
    //asm!("msr MSP_NS, {}",
    //       in(reg) ns_estack);
    //let one = 1;
    //asm!("bics {0}, {1}
    //      bxns {0}", in(reg) ns_start, in(reg) one);
    //loop {}

    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf53_app_peripherals.clock.low_stop();
    nrf53_app_peripherals.clock.high_stop();

    // TODO: LFXO requires specific GPIO/oscillator setup....
    nrf53_app_peripherals
        .clock
        .low_set_source(nrf53::clock::LowClockSource::LFRC);
    nrf53_app_peripherals.clock.low_start();
    nrf53_app_peripherals
        .clock
        .high_set_source(nrf53::clock::HighClockSource::HFXO);
    nrf53_app_peripherals.clock.high_start();
    while !nrf53_app_peripherals.clock.low_started() {}
    while !nrf53_app_peripherals.clock.high_started() {}

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::rr_component_helper!(NUM_PROCS));

    let platform = Platform {
        alarm,
        button,
        pconsole,
        console,
        led,
        gpio,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        scheduler,
        systick: cortexm33::systick::SysTick::new_with_calibration(64000000),
    };

    let _ = platform.pconsole.start();

    debug!("Initialization complete. Entering main loop\r");

    /// These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            &_sapps as *const u8,
            &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
        ),
        core::slice::from_raw_parts_mut(
            &mut _sappmem as *mut u8,
            &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
        ),
        &mut PROCESSES,
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}
