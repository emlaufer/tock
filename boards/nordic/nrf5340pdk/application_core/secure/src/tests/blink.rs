use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use kernel::debug;
use kernel::hil::gpio::{Client, Configure, Interrupt, InterruptEdge, Output};
use kernel::hil::time::{Alarm, AlarmClient, Frequency};
use kernel::static_init;
use nrf53::gpio;
use nrf53::rtc::Rtc;

const WAIT_MS: u32 = 500;

pub unsafe fn run(
    mux_alarm: &'static MuxAlarm<'static, Rtc>,
    led_pin: gpio::Pin,
    button_pin: gpio::Pin,
    peripherals: &'static nrf53::chip::Nrf53AppDefaultPeripherals<'static>
) {
    let blink = static_init!(
        Blink<VirtualMuxAlarm<'static, Rtc>>,
        Blink::new(VirtualMuxAlarm::new(mux_alarm), led_pin, button_pin, peripherals)
    );

    blink.alarm.set_alarm_client(blink);
    blink.button.set_client(blink);
    blink.run();
}

struct Blink<A: Alarm<'static>> {
    alarm: A,
    led: &'static gpio::GPIOPin<'static>,
    button: &'static gpio::GPIOPin<'static>,
}

impl<A: Alarm<'static>> Blink<A> {
    fn new(alarm: A, led_pin: gpio::Pin, button_pin: gpio::Pin, peripherals: &'static nrf53::chip::Nrf53AppDefaultPeripherals<'static>) -> Blink<A> {
        let led = &peripherals.gpio_port[led_pin];
        let button = &peripherals.gpio_port[button_pin];

        led.make_output();
        button.make_input();
        button.enable_interrupts(InterruptEdge::FallingEdge);

        Blink { alarm, led, button }
    }

    fn run(&self) {
        self.led.toggle();

        let interval = WAIT_MS * <A::Frequency>::frequency() / 1000;
        self.alarm
            .set_alarm(self.alarm.now(), A::Ticks::from(interval));
    }
}

impl<A: Alarm<'static>> AlarmClient for Blink<A> {
    fn alarm(&self) {
        debug!("blink");
        self.run();
    }
}

impl<A: Alarm<'static>> Client for Blink<A> {
    fn fired(&self) {
        debug!("CLICK");
        self.run();
    }
}
