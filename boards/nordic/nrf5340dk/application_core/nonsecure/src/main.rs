#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![feature(asm, asm_sym)]
#![feature(cmse_nonsecure_entry)]
#![feature(abi_c_cmse_nonsecure_call)]
#![feature(global_asm)]
#![feature(naked_functions)]

use kernel::{static_init};
use kernel::component::Component;
use kernel::hil::led::LedLow;
use kernel::hil::time::Counter;
use nrf53::gpio::Pin;
use nrf53::chip::Nrf53NSAppDefaultPeripherals;

/// Debug Writer
pub mod io;
pub mod crt1;

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

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 8;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x800] = [0; 0x800];

#[inline(never)]
unsafe fn get_peripherals() -> &'static mut Nrf53NSAppDefaultPeripherals<'static> {
    // Init chip peripheral drivers
    let nrf53_app_peripherals = static_init!(
        Nrf53NSAppDefaultPeripherals,
        Nrf53NSAppDefaultPeripherals::new()
    );

    nrf53_app_peripherals
}

extern "C" {
    fn test_gate_veneer();
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    let nrf53_app_peripherals = get_peripherals();

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    // interesting point ... now there are two peripheral structs too...
    // TODO: who should handle gpio interrupts?
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

    // TODO: where should this live? uar
    nrf53_app_peripherals.uarte0.initialize(
        nrf53::pinmux::Pinmux::new(UART_TXD as u32),
        nrf53::pinmux::Pinmux::new(UART_RXD as u32),
        UART_CTS.map(|x| nrf53::pinmux::Pinmux::new(x as u32)),
        UART_RTS.map(|x| nrf53::pinmux::Pinmux::new(x as u32)),
    );

    // Create a shared UART channel for the console and for kernel debug.
    /*let uart_mux =
        components::console::UartMuxComponent::new(uart_channel, 115200, dynamic_deferred_caller)
            .finalize(());*/

    //debug_sync!("HI THERE");

    use kernel::hil::gpio::Configure;
    use kernel::hil::gpio::Output;
    let led = &nrf53_app_peripherals.gpio_port[LED2_PIN];
    led.make_output();
    led.clear();

    test_gate_veneer();
    loop {};

    //let button = components::button::ButtonComponent::new(
    //    board_kernel,
    //    capsules::button::DRIVER_NUM,
    //    components::button_component_helper!(
    //        nrf53::gpio::GPIOPin,
    //        (
    //            &nrf53_app_peripherals.gpio_port[BUTTON1_PIN],
    //            kernel::hil::gpio::ActivationMode::ActiveLow,
    //            kernel::hil::gpio::FloatingState::PullUp
    //        ), //23
    //        (
    //            &nrf53_app_peripherals.gpio_port[BUTTON2_PIN],
    //            kernel::hil::gpio::ActivationMode::ActiveLow,
    //            kernel::hil::gpio::FloatingState::PullUp
    //        ), //24
    //        (
    //            &nrf53_app_peripherals.gpio_port[BUTTON3_PIN],
    //            kernel::hil::gpio::ActivationMode::ActiveLow,
    //            kernel::hil::gpio::FloatingState::PullUp
    //        ), //8
    //        (
    //            &nrf53_app_peripherals.gpio_port[BUTTON4_PIN],
    //            kernel::hil::gpio::ActivationMode::ActiveLow,
    //            kernel::hil::gpio::FloatingState::PullUp
    //        ) //9
    //    ),
    //)
    //.finalize(components::button_component_buf!(nrf53::gpio::GPIOPin));

    //let led = components::led::LedsComponent::new().finalize(components::led_component_helper!(
    //    LedLow<'static, nrf53::gpio::GPIOPin>,
    //    LedLow::new(&nrf53_app_peripherals.gpio_port[LED1_PIN]),
    //    LedLow::new(&nrf53_app_peripherals.gpio_port[LED2_PIN]),
    //    LedLow::new(&nrf53_app_peripherals.gpio_port[LED3_PIN]),
    //    LedLow::new(&nrf53_app_peripherals.gpio_port[LED4_PIN]),
    //));

    core::ptr::write_volatile(0x40842518 as *mut u32, 1 << 30);
    core::ptr::write_volatile(0x4084250C as *mut u32, 1 << 30);
    loop {}
}
