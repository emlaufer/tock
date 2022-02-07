use core::fmt::Write;
use core::panic::PanicInfo;
use cortexm33;
use kernel::debug;
use kernel::debug::IoWrite;
use kernel::hil::led;
use kernel::hil::uart::{self, Configure};
use nrf53::gpio::Pin;

use crate::CHIP;
use crate::PROCESSES;
use crate::PROCESS_PRINTER;

enum Writer {
    WriterUart(/* initialized */ bool),
    WriterRtt(&'static capsules::segger_rtt::SeggerRttMemory<'static>),
}

static mut WRITER: Writer = Writer::WriterUart(false);

fn wait() {
    for _ in 0..100 {
        cortexm33::support::nop();
    }
}

/// Set the RTT memory buffer used to output panic messages.
pub unsafe fn set_rtt_memory(
    rtt_memory: &'static mut capsules::segger_rtt::SeggerRttMemory<'static>,
) {
    WRITER = Writer::WriterRtt(rtt_memory);
}

impl Write for Writer {
    fn write_str(&mut self, s: &str) -> ::core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

impl IoWrite for Writer {
    fn write(&mut self, buf: &[u8]) {
        match self {
            Writer::WriterUart(ref mut initialized) => {
                // Here, we create a second instance of the Uarte struct.
                // This is okay because we only call this during a panic, and
                // we will never actually process the interrupts
                //let uart = nrf53::uart::Uarte::new(nrf53::uart::UARTE0_BASE_NONSECURE);
                let uart = nrf53::uart::Uarte::new(nrf53::uart::UARTE0_BASE_SECURE);
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
            Writer::WriterRtt(rtt_memory) => {
                let up_buffer = unsafe { &*rtt_memory.get_up_buffer_ptr() };
                let buffer_len = up_buffer.length.get();
                let buffer = unsafe {
                    core::slice::from_raw_parts_mut(
                        up_buffer.buffer.get() as *mut u8,
                        buffer_len as usize,
                    )
                };

                let mut write_position = up_buffer.write_position.get();

                for &c in buf {
                    buffer[write_position as usize] = c;
                    write_position = (write_position + 1) % buffer_len;
                    up_buffer.write_position.set(write_position);
                    wait();
                }
            }
        };
    }
}

#[cfg(not(test))]
#[inline(never)]
#[no_mangle]
#[panic_handler]
/// Panic handler
pub unsafe extern "C" fn panic_fmt(pi: &PanicInfo) -> ! {
        core::ptr::write_volatile(0x50842518 as *mut u32, 1 << 30);
        core::ptr::write_volatile(0x5084250C as *mut u32, 1 << 30);
    const LED2_PIN: Pin = Pin::P0_28;
    let led_kernel_pin = &nrf53::gpio::GPIOPin::new(LED2_PIN, nrf53::gpio::GPIO_BASE_ADDRESS_SECURE, nrf53::gpio::GPIOTE0_BASE);
    let led = &mut led::LedLow::new(led_kernel_pin);
    let writer = &mut WRITER;
    debug::panic(
        &mut [led],
        writer,
        pi,
        &cortexm33::support::nop,
        &PROCESSES,
        &CHIP,
        &PROCESS_PRINTER
    )
}
