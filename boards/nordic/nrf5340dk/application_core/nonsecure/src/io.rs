use core::panic::PanicInfo;
use core::fmt::{write, Arguments, Result, Write};
use kernel::debug::IoWrite;
use kernel::hil::uart::{self, Configure};

#[macro_export]
macro_rules! debug_sync {
    () => ({
        // Allow an empty debug!() to print the location when hit
        debug_sync!("")
    });
    ($msg:expr $(,)?) => ({
        $crate::io::debug_println(format_args!($msg))
    });
    ($fmt:expr, $($arg:tt)+) => ({
        $crate::io::debug_println(format_args!($fmt, $($arg)+))
    });
}

pub fn debug_println(args: Arguments) {
    let writer = unsafe { &mut WRITER };

    let _ = write(writer, args);
    let _ = writer.write_str("\r\n");
}

struct Writer(bool);

static mut WRITER: Writer = Writer(false);

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) {
                // Here, we create a second instance of the Uarte struct.
                // This is okay because we only call this during a panic, and
                // we will never actually process the interrupts
                let uart = nrf53::uart::Uarte::new(nrf53::uart::UARTE0_BASE_NONSECURE);
                let initialized = &mut self.0;
                if !*initialized {
                    *initialized = true;
                    let _ = uart.configure(uart::Parameters {
                        baud_rate: 115200,
                        stop_bits: uart::StopBits::One,
                        parity: uart::Parity::None,
                        hw_flow_control: false,
                        width: uart::Width::Eight,
                    });
                    /*unsafe {
                    uart.initialize(
                        nrf53::pinmux::Pinmux::new(Pin::P0_20 as u32),
                        nrf53::pinmux::Pinmux::new(Pin::P0_22 as u32),
                        None,
                        None,
                    );
                    }*/
                }
                for &c in buf {
                    unsafe {
                        uart.send_byte(c);
                    }
                    while !uart.tx_ready() {}
                }
            }
}

// TODO: should this call into a secure gate, to then trigger the secure panic handler?
#[cfg(not(test))]
#[no_mangle]
#[panic_handler]
/// Panic handler
pub unsafe extern "C" fn panic_fmt(pi: &PanicInfo) -> ! {
    /*const LED2_PIN: Pin = Pin::P0_28;
    let led_kernel_pin = &nrf53::gpio::GPIOPin::new(LED2_PIN, nrf53::gpio::GPIO_BASE_ADDRESS_NONSECURE, nrf53::gpio::GPIOTE1_BASE);
    let led = &mut led::LedLow::new(led_kernel_pin);
    led.make_output();
    led.clear();*/
    loop {}
}
