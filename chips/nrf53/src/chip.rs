use crate::deferred_call_tasks::DeferredCallTask;
use core::fmt::Write;
use cortexm33::{self, nvic};
use kernel::deferred_call;
use kernel::platform::chip::InterruptService;

pub struct NRF53<'a, I: InterruptService<DeferredCallTask> + 'a> {
    mpu: cortexm33::mpu::MPU,
    userspace_kernel_boundary: cortexm33::syscall::SysCall,
    scheduler_timer: cortexm33::systick::SysTick,
    // TODO: should this be a field here or in the peripherals struct?
    pub sau: cortexm33::sau::SAU,
    interrupt_service: &'a I,
}

impl<'a, I: InterruptService<DeferredCallTask> + 'a> NRF53<'a, I> {
    pub unsafe fn new(interrupt_service: &'a I) -> NRF53<I> {
        NRF53 {
            mpu: cortexm33::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm33::syscall::SysCall::new(),
            // The NRF53's systick is uncalibrated, but is clocked from the
            // 64Mhz CPU clock.
            scheduler_timer: cortexm33::systick::SysTick::new_with_calibration(64000000),
            sau: cortexm33::sau::SAU::new(),
            interrupt_service,
        }
    }
}

pub struct Nrf53AppDefaultPeripherals<'a> {
    // TODO: is this technically a peripheral??
    pub gpio_port: crate::gpio::Port<'a, { crate::gpio::NUM_PINS }>,
    pub pwr_clk: crate::power::Power<'a>,
    pub rtc0: crate::rtc::Rtc<'a>,
    pub uarte0: crate::uart::Uarte<'a>,
    pub spi0: crate::spi::Spi,
    pub spu: crate::spu::Spu,
    pub clock: crate::clock::Clock,
    pub nvmc: crate::nvmc::Nvmc,
}

impl<'a> Nrf53AppDefaultPeripherals<'a> {
    pub fn new() -> Self {
        Self {
            gpio_port: crate::gpio::nrf53_gpio_create(),
            pwr_clk: crate::power::Power::new(crate::power::POWER_BASE_SECURE),
            rtc0: crate::rtc::Rtc::new(crate::rtc::RTC0_BASE_SECURE),
            uarte0: crate::uart::Uarte::new(crate::uart::UARTE0_BASE_SECURE),
            spi0: crate::spi::Spi::new(crate::spi::SECURE_INSTANCES[0]),
            spu: crate::spu::Spu::new(),
            clock: crate::clock::Clock::new(crate::clock::CLOCK_BASE_SECURE),
            nvmc: crate::nvmc::Nvmc::new(crate::nvmc::NVMC_BASE_SECURE),
        }
    }
}

impl<'a> kernel::platform::chip::InterruptService<DeferredCallTask>
    for Nrf53AppDefaultPeripherals<'a>
{
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            crate::app_peripheral_ids::GPIOTE0 => self.gpio_port.handle_interrupt(),
            crate::app_peripheral_ids::POWER_CLOCK_RESET => self.pwr_clk.handle_interrupt(),
            crate::app_peripheral_ids::RTC0 => self.rtc0.handle_interrupt(),
            // TODO: forward to proper peripheral
            crate::app_peripheral_ids::SPI_TWI_UARTE0 => {
                match (self.spi0.is_enabled(), self.uarte0.is_enabled()) {
                    (false, false) => (),
                    (true, false) => self.spi0.handle_interrupt(),
                    (false, true) => self.uarte0.handle_interrupt(),
                    (true, true) => debug_assert!(
                        false,
                        "SPI0 and UARTE0 cannot be \
                         enabled at the same time."
                    ),
                }
            }
            //crate::app_peripheral_ids::SPU => // TODO
            _ => return false,
        }
        true
    }

    unsafe fn service_deferred_call(&self, task: DeferredCallTask) -> bool {
        match task {
            DeferredCallTask::Nvmc => self.nvmc.handle_interrupt(),
        }
        true
    }
}

impl<'a, I: InterruptService<DeferredCallTask> + 'a> kernel::platform::chip::Chip for NRF53<'a, I> {
    type MPU = cortexm33::mpu::MPU;
    type UserspaceKernelBoundary = cortexm33::syscall::SysCall;

    fn mpu(&self) -> &Self::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &Self::UserspaceKernelBoundary {
        &self.userspace_kernel_boundary
    }

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(task) = deferred_call::DeferredCall::next_pending() {
                    if !self.interrupt_service.service_deferred_call(task) {
                        panic!("unhandled deferred call task");
                    }
                } else if let Some(interrupt) = nvic::next_pending() {
                    if !self.interrupt_service.service_interrupt(interrupt) {
                        panic!("unhandled interrupt {}", interrupt);
                    }
                    let n = nvic::Nvic::new(interrupt);
                    n.clear_pending();
                    n.enable();
                } else {
                    break;
                }
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { nvic::has_pending() || deferred_call::has_tasks() }
    }

    fn sleep(&self) {
        unsafe {
            cortexm33::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm33::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        cortexm33::print_cortexm33_state(write);
    }
}
