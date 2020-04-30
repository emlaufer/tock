/// System Protection Unit (SPU) for the nRF53.
/// NOTE: Quick and dirty first pass at making a driver, needs a lot of work.
use crate::gpio;
use kernel::common::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::common::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::common::StaticRef;
use kernel::debug;

const SPU_BASE: StaticRef<SpuRegisters> =
    unsafe { StaticRef::new(0x50003000 as *const SpuRegisters) };

pub static mut SPU: Spu = Spu::new();

/// XXX Errata 19: 32 regions of 32 KiB for early nRF5340-PDK, 64 regions of 16 KiB on corrected chips
pub const NUM_FLASH_REGIONS: usize = 32;
pub const NUM_RAM_REGIONS: usize = 64;
const NUM_PERIPHIDS: usize = 256;

register_structs! {
    SpuRegisters {
        (0x000 => _reserved0),
        /// A security violation has been detected for the RAM memory space.
        (0x100 => events_ramaccerr: ReadWrite<u32, Event::Register>),
        /// A security violation has been detected for the flash memory space.
        (0x104 => events_flashaccerr: ReadWrite<u32, Event::Register>),
        /// A security violation has been detected on one or several peripherals.
        (0x108 => events_periphaccerr: ReadWrite<u32, Event::Register>),
        (0x10C => _reserved1),
        /// Publish configuration for events_ramaccerr.
        (0x180 => publish_ramaccerr: ReadWrite<u32, Publish::Register>),
        /// Publish configuration for events_flashaccerr.
        (0x184 => publish_flashaccerr: ReadWrite<u32, Publish::Register>),
        /// Publish configuration for events_periphaccerr.
        (0x188 => publish_periphaccerr: ReadWrite<u32, Publish::Register>),
        (0x18C => _reserved2),
        /// Enable or disable interrupt.
        (0x300 => inten: ReadWrite<u32, Interrupt::Register>),
        /// Enable interrupt.
        (0x304 => intenset: ReadWrite<u32, Interrupt::Register>),
        /// Disable interrupt.
        (0x308 => intenclr: ReadWrite<u32, Interrupt::Register>),
        (0x30C => _reserved3),
        /// Show implemented features for the current device.
        (0x400 => cap: ReadOnly<u32, Cap::Register>),
        /// Configure bits to lock down CPU features at runtime.
        /// XXX Errata 22: CPULOCK register not functional on early nRF5340-PDK.
        (0x404 => cpulock: ReadWrite<u32, CpuLock::Register>),
        (0x408 => _reserved4),
        /// Access for bus access generated from the external domain n.
        /// List capabilities of the external domain n.
        (0x440 => extdomain_perm: ReadWrite<u32, ExtDomain::Register>),
        (0x444 => _reserved5),
        /// Select between secure and non-secure attribute for the DPPI channels.
        (0x480 => dppi_perm: ReadWrite<u32>),
        /// Prevent further modification of dppi_perm.
        (0x484 => dppi_lock: ReadWrite<u32, Lock::Register>),
        (0x488 => _reserved6),
        /// Select between secure and non-secure attribute for pins of GPIO port 0.
        (0x4C0 => gpioport_0_perm: ReadWrite<u32>),
        /// Prevent further modification of gpioport_0_perm.
        (0x4C4 => gpioport_0_lock: ReadWrite<u32, Lock::Register>),
        /// Select between secure and non-secure attribute for pins of GPIO port 1.
        (0x4C8 => gpioport_1_perm: ReadWrite<u32>),
        /// Prevent further modification of gpioport_1_perm.
        (0x4CC => gpioport_1_lock: ReadWrite<u32, Lock::Register>),
        (0x4D0 => _reserved7),
        /// Define which flash region can contain the non-secure callable (NSC) region 0.
        (0x500 => flashnsc_0_region: ReadWrite<u32, Region::Register>),
        /// Define the size of the non-secure callable (NSC) flash region 0.
        (0x504 => flashnsc_0_size: ReadWrite<u32, Size::Register>),
        /// Define which flash region can contain the non-secure callable (NSC) region 1.
        (0x508 => flashnsc_1_region: ReadWrite<u32, Region::Register>),
        /// Define the size of the non-secure callable (NSC) flash region 1.
        (0x50C => flashnsc_1_size: ReadWrite<u32, Size::Register>),
        (0x510 => _reserved8),
        /// Define which RAM region can contain the non-secure callable (NSC) region 0.
        (0x540 => ramnsc_0_region: ReadWrite<u32, Region::Register>),
        /// Define the size of the non-secure callable (NSC) RAM region 0.
        (0x544 => ramnsc_0_size: ReadWrite<u32, Size::Register>),
        /// Define which RAM region can contain the non-secure callable (NSC) region 1.
        (0x548 => ramnsc_1_region: ReadWrite<u32, Region::Register>),
        /// Define the size of the non-secure callable (NSC) RAM region 1.
        (0x54C => ramnsc_1_size: ReadWrite<u32, Size::Register>),
        (0x550 => _reserved9),
        /// Access permissions for flash region n.
        (0x600 => flashregion_perm: [ReadWrite<u32, RegionPerm::Register>; NUM_FLASH_REGIONS]),
        (0x680 => _reserved10), // XXX: Errata #19
        /// Access permissions for RAM region n.
        (0x700 => ramregion_perm: [ReadWrite<u32, RegionPerm::Register>; NUM_RAM_REGIONS]),
        /// List capabilities and access permisions for the peripheral with ID n.
        (0x800 => periphid_perm: [ReadWrite<u32, PeriphIdPerm::Register>; NUM_PERIPHIDS]),
        (0xC00 => @END),
    }
}

register_bitfields! [u32,
    Event [
        READY OFFSET(0) NUMBITS(1)
    ],

    Publish [
        CHIDX OFFSET(0) NUMBITS(8),
        ENABLE OFFSET(31) NUMBITS(1)
    ],

    Interrupt [
        RAMACCERR 0,
        FLASHACCERR 1,
        PERIPHACCERR 2
    ],

    Cap [
        /// Show Arm TrustZone status.
        TZM OFFSET(0) NUMBITS(1) [
            /// Arm TrustZone support not available.
            NotAvailable = 0,
            /// Arm TrustZone support is available.
            Enabled = 1
        ]
    ],

    /// Write '1' to any position to set the corresponding lock bit, which will remain set until
    /// the next reset. Any '0' writes to this register will be ignored.
    CpuLock [
        /// Lock prevents updating the secure interrupt configuration and changing:
        /// * The secure vector table base address
        /// * Handling of secure interrupt priority
        /// * BusFault, HardFault, and NMI security target
        /// until next reset. CANNOT BE UNLOCKED!
        LOCKSVTAIRCR OFFSET(0) NUMBITS(1) [
            Unlocked = 0,
            /// Disables writes to the VTOR_S, AIRCR.PRIS, and AIRCR.BFHNMINS registers.
            Locked = 1
        ],
        /// Lock prevents updating the non-secure vector table base address until the next reset.
        /// CANNOT BE UNLOCKED!
        LOCKNSVTOR OFFSET(1) NUMBITS(1) [
            Unlocked = 0,
            /// Disables writes to VTOR_NS.
            Locked = 1
        ],
        /// Lock prevents updating the secure MPU regions until the next reset. CANNOT BE UNLOCKED!
        LOCKSMPU OFFSET(2) NUMBITS(1) [
            Unlocked = 0,
            /// Disables writes to MPU_CTRL, MPU_RNR, MPU_RBAR, MPU_RLAR, MPU_RBAR_An, and
            /// MPU_RLAR_An.
            Locked = 1
        ],
        /// Lock prevents updating the non-secure MPU regions until next reset. CANNOT BE UNLOCKED!
        LOCKNSMPU OFFSET(3) NUMBITS(1) [
            Unlocked = 0,
            /// Disables writes to MPU_CTRL_NS, MPU_RNR_NS, MPU_RBAR_NS, MPU_RLAR_NS,
            /// MPU_RBAR_A_NSn, and MPU_RLAR_A_NSn.
            Locked = 1
        ],
        /// Lock prevents updating the secure SAU regions until the next reset. CANNOT BE UNLOCKED!
        LOCKSAU OFFSET(4) NUMBITS(1) [
            Unlocked = 0,
            /// Disables writes to SAU_CTRL, SAU_RNR, SAU_RBAR, AND SAU_RLAR.
            Locked = 1
        ]
    ],

    ExtDomain [
        /// Define configuration capabilities for TrustZone Cortex-M secure attribute. This does
        /// not affect DPPI in the external domain.
        /// READ ONLY!
        SECUREMAPPING OFFSET(0) NUMBITS(2) [
            /// The bus accesses from this external domain always have the non-secure attribute
            /// set.
            NonSecure = 0,
            /// The bus accesses from this external domain always have the secure attribute set.
            Secure = 1,
            /// Non-secure or secure attribute for bus access from this domain is defined by
            /// SECATTR.
            UserSelectable = 2
        ],
        /// Peripheral security mapping. In effect only if SECUREMAPPING == UserSelectable.
        SECATTR OFFSET(4) NUMBITS(1) [
            NonSecure = 0,
            Secure = 1
        ],
        /// Lock this register until next reset. CANNOT BE UNLOCKED!
        LOCK OFFSET(8) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ]
    ],

    Lock [
        LOCK OFFSET(0) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ]
    ],

    Region [
        /// Region number.
        REGION OFFSET(0) NUMBITS(6) [],
        /// Lock this register until next reset.
        LOCK OFFSET(8) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ]
    ],

    Size [
        /// Size of the non-secure callable (NSC) region n.
        SIZE OFFSET(0) NUMBITS(4) [
            Disabled = 0,
            Bytes32 = 1,
            Bytes64 = 2,
            Bytes128 = 3,
            Bytes256 = 4,
            Bytes512 = 5,
            Bytes1024 = 6,
            Bytes2048 = 7,
            Bytes4096 = 8
        ],
        /// Lock this register until next reset.
        LOCK OFFSET(8) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ]
    ],

    RegionPerm [
        /// Enable or disable instruction fetches from flash region n.
        EXECUTE OFFSET(0) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ],
        /// Enable or disable write permissions for flash region n.
        WRITE OFFSET(1) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ],
        /// Enable or disable read permissions for flash region n.
        READ OFFSET(2) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ],
        /// Security attribute for flash region n.
        SECATTR OFFSET(4) NUMBITS(1) [
            NonSecure = 0,
            Secure = 1
        ],
        /// Lock this register until next reset.
        LOCK OFFSET(8) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ]
    ],

    PeriphIdPerm [
        /// Define configuration capabilities for Arm TrustZone Cortex-M secure attribute.
        /// READ ONLY!
        SECUREMAPPING OFFSET(0) NUMBITS(2) [
            NonSecure = 0,
            Secure = 1,
            UserSelectable = 2,
            Split = 3
        ],
        /// Indicates if the peripheral has DMA capabilities and if DMA transfer can be assigned to
        /// a different security attribute than the peripheral itself.
        /// READ ONLY!
        DMA OFFSET(2) NUMBITS(2) [
            NoDMA = 0,
            NoSeparateAttribute = 1,
            SeparateAttribute = 2
        ],
        /// Peripheral security mapping. This bit has effect only if SECUREMAPPING reads as
        /// UserSelectable or Split.
        SECATTR OFFSET(4) NUMBITS(1) [
            /// If SECUREMAPPING == UserSelectable: Mapped in non-secure address space.
            /// If SECUREMAPPING == Split: Mapped in non-secure and secure address space.
            NonSecure = 0,
            /// Mapped exclusively in secure address space.
            Secure = 1
        ],
        /// Security attributeion for the DMA transfer. This bit has effect only if SECATTR is set
        /// to Secure.
        DMASEC OFFSET(5) NUMBITS(1) [
            NonSecure = 0,
            Secure = 1
        ],
        /// Lock this register until next reset.
        LOCK OFFSET(8) NUMBITS(1) [
            Unlocked = 0,
            Locked = 1
        ],
        /// Indicate if this peripheral is present.
        PRESENT OFFSET(31) NUMBITS(1) [
            NotPresent = 0,
            Present = 1
        ]
    ]
];

// Region permissions
pub const PERM_NONE: u32 = 0;
pub const PERM_EXECUTE: u32 = 1;
pub const PERM_WRITE: u32 = 2;
pub const PERM_READ: u32 = 4;
pub const PERM_SECURE: u32 = 16;

#[derive(Debug)]
pub enum RegionType {
    Flash,
    Ram,
}

#[derive(Debug)]
pub enum SecureMapping {
    NonSecure = 0,
    Secure = 1,
    UserSelectable = 2,
    Split = 3,
}

#[derive(Clone, Copy, Debug)]
pub enum Error {
    InvalidRegionNum,
    InvalidRegionSize,
    OutOfRegions,
    SecureMappingFixed,
    PeripheralNotPresent,
}

pub struct Spu {
    registers: StaticRef<SpuRegisters>,
    allocated_flash_nsc: [bool; 2],
    allocated_ram_nsc: [bool; 2],
}

impl Spu {
    const fn new() -> Self {
        Spu {
            registers: SPU_BASE,
            allocated_flash_nsc: [false; 2],
            allocated_ram_nsc: [false; 2],
        }
    }

    /// Initialize SPU, returning whether or not TrustZone is supported and the SPU was able to
    /// start.
    pub fn init(&self) -> bool {
        self.registers.cap.matches_all(Cap::TZM::Enabled)
    }

    /// Lock all configuration registers (effective until next reset).
    pub fn commit(&self) {
        let registers = &self.registers;

        // CPU.
        registers.cpulock.modify(
            CpuLock::LOCKSVTAIRCR::Locked
                + CpuLock::LOCKNSVTOR::Locked
                + CpuLock::LOCKSMPU::Locked
                + CpuLock::LOCKNSMPU::Locked
                + CpuLock::LOCKSAU::Locked,
        );

        // External domains (network core).
        registers.extdomain_perm.modify(ExtDomain::LOCK::Locked);

        // DPPI and GPIO.
        registers.dppi_lock.modify(Lock::LOCK::Locked);
        registers.gpioport_0_lock.modify(Lock::LOCK::Locked);
        registers.gpioport_1_lock.modify(Lock::LOCK::Locked);

        // Flash NSC regions.
        registers.flashnsc_0_region.modify(Region::LOCK::Locked);
        registers.flashnsc_0_size.modify(Size::LOCK::Locked);
        registers.flashnsc_1_region.modify(Region::LOCK::Locked);
        registers.flashnsc_1_size.modify(Size::LOCK::Locked);

        // RAM NSC regions.
        registers.ramnsc_0_region.modify(Region::LOCK::Locked);
        registers.ramnsc_0_size.modify(Size::LOCK::Locked);
        registers.ramnsc_1_region.modify(Region::LOCK::Locked);
        registers.ramnsc_1_size.modify(Size::LOCK::Locked);

        // Flash region permissions.
        for i in 0..NUM_FLASH_REGIONS {
            registers.flashregion_perm[i].modify(RegionPerm::LOCK::Locked);
        }

        // RAM region permissions.
        for i in 0..NUM_RAM_REGIONS {
            registers.ramregion_perm[i].modify(RegionPerm::LOCK::Locked);
        }

        // Peripheral permissions.
        for i in 0..NUM_PERIPHIDS {
            registers.periphid_perm[i].modify(PeriphIdPerm::LOCK::Locked);
        }
    }

    /// Gets the secure mapping of the network core.
    pub fn get_net_core_sec(&self) -> SecureMapping {
        match self
            .registers
            .extdomain_perm
            .read_as_enum(ExtDomain::SECUREMAPPING)
        {
            Some(ExtDomain::SECUREMAPPING::Value::NonSecure) => SecureMapping::NonSecure,
            Some(ExtDomain::SECUREMAPPING::Value::Secure) => SecureMapping::Secure,
            Some(ExtDomain::SECUREMAPPING::Value::UserSelectable) => SecureMapping::UserSelectable,
            None => unreachable!(),
        }
    }

    /// Sets the secure mapping of the network core. Returns Err() if the security mapping cannot
    /// be changed.
    pub fn set_net_core_sec(&self, secure: bool) -> Result<(), ()> {
        if let Some(ExtDomain::SECUREMAPPING::Value::UserSelectable) = self
            .registers
            .extdomain_perm
            .read_as_enum(ExtDomain::SECUREMAPPING)
        {
            let secattr = match secure {
                true => ExtDomain::SECATTR::Secure,
                false => ExtDomain::SECATTR::NonSecure,
            };
            self.registers.extdomain_perm.write(secattr);

            Ok(())
        } else {
            Err(())
        }
    }

    /// Gets GPIO pin security.
    pub fn get_gpio_sec(&self, gpio: gpio::Pin) -> bool {
        let pin_num = gpio as usize;

        // 32 pins per register.
        let gpio_reg = match pin_num / 32 {
            0 => &self.registers.gpioport_0_perm,
            1 => &self.registers.gpioport_1_perm,
            _ => unreachable!(),
        };

        // Mask for pin's security bit.
        (gpio_reg.get() >> (pin_num % 32)) & 1 == 1
    }

    /// Sets GPIO pin security.
    pub fn set_gpio_sec(&self, gpio: gpio::Pin, secure: bool) {
        let pin_num = gpio as usize;

        // 32 pins per register.
        let gpio_reg = match pin_num / 32 {
            0 => &self.registers.gpioport_0_perm,
            1 => &self.registers.gpioport_1_perm,
            _ => unreachable!(),
        };

        // Generate new register value, updating pin's security.
        let reg_val = gpio_reg.get();
        let mask: u32 = 1 << (pin_num % 32);
        let reg_val = match secure {
            true => reg_val | mask,
            false => reg_val & !mask,
        };

        gpio_reg.set(reg_val);
    }

    /// Get flash region permissions.
    pub fn get_flash_region_perms(&self, region_num: usize) -> Result<u32, Error> {
        // Make sure region_num is within bounds.
        if region_num >= NUM_FLASH_REGIONS {
            return Err(Error::InvalidRegionNum);
        }

        // Return valid permissions.
        Ok(self.registers.flashregion_perm[region_num].get() & 0b10111)
    }

    /// Get RAM region permissions.
    pub fn get_ram_region_perms(&self, region_num: usize) -> Result<u32, Error> {
        // Make sure region_num is within bounds.
        if region_num >= NUM_RAM_REGIONS {
            return Err(Error::InvalidRegionNum);
        }

        // Return valid permissions.
        Ok(self.registers.ramregion_perm[region_num].get() & 0b10111)
    }

    /// Set flash region permissions.
    pub fn set_flash_region_perms(&self, region_num: usize, perms: u32) -> Result<(), Error> {
        // Make sure region_num is within bounds.
        if region_num >= NUM_FLASH_REGIONS {
            return Err(Error::InvalidRegionNum);
        }

        // Apply legal permissions.
        let perms = perms & 0b10111;
        self.registers.flashregion_perm[region_num].set(perms);
        Ok(())
    }

    /// Set RAM region permissions.
    pub fn set_ram_region_perms(&self, region_num: usize, perms: u32) -> Result<(), Error> {
        // Make sure region_num is within bounds.
        if region_num >= NUM_RAM_REGIONS {
            return Err(Error::InvalidRegionNum);
        }

        // Apply legal permissions.
        let perms = perms & 0b10111;
        self.registers.ramregion_perm[region_num].set(perms);
        Ok(())
    }

    /// Add a new NSC region. Returns the NSC region number on success.
    pub fn add_nsc_region(
        &mut self,
        region_type: RegionType,
        region_num: usize,
        size: usize,
    ) -> Result<usize, Error> {
        let allocated_nsc = match region_type {
            RegionType::Flash => &mut self.allocated_flash_nsc,
            RegionType::Ram => &mut self.allocated_ram_nsc,
        };

        // Allocate an NSC region, if available.
        let mut nsc = Err(Error::OutOfRegions);
        for i in 0..allocated_nsc.len() {
            if !allocated_nsc[i] {
                allocated_nsc[i] = true;
                nsc = Ok(i);
                break;
            }
        }

        // Configure NSC region.
        if nsc.is_ok() {
            let nsc_num = nsc.unwrap();

            // XXX: need to find a cleaner way to get these regs.
            let (region_reg, size_reg) = match region_type {
                RegionType::Flash => match nsc_num {
                    0 => (
                        &self.registers.flashnsc_0_region,
                        &self.registers.flashnsc_0_size,
                    ),
                    1 => (
                        &self.registers.flashnsc_1_region,
                        &self.registers.flashnsc_1_size,
                    ),
                    _ => unreachable!(),
                },
                RegionType::Ram => match nsc_num {
                    0 => (
                        &self.registers.ramnsc_0_region,
                        &self.registers.ramnsc_0_size,
                    ),
                    1 => (
                        &self.registers.ramnsc_1_region,
                        &self.registers.ramnsc_1_size,
                    ),
                    _ => unreachable!(),
                },
            };

            // Convert size from number to register value.
            let size = match size {
                32 => Size::SIZE::Bytes32,
                64 => Size::SIZE::Bytes64,
                128 => Size::SIZE::Bytes128,
                256 => Size::SIZE::Bytes256,
                512 => Size::SIZE::Bytes512,
                1024 => Size::SIZE::Bytes1024,
                2048 => Size::SIZE::Bytes2048,
                4096 => Size::SIZE::Bytes4096,
                _ => {
                    debug!("Invalid NSC region size ({})", size);
                    return Err(Error::InvalidRegionSize);
                }
            };

            // Write region number and size.
            region_reg.write(Region::REGION.val(region_num as u32));
            size_reg.write(size);
        }

        nsc
    }

    /// Remove an existing NSC region.
    pub fn rm_nsc_region(&mut self, region_type: RegionType, nsc_num: usize) {
        let allocated_nsc = match region_type {
            RegionType::Flash => &mut self.allocated_flash_nsc,
            RegionType::Ram => &mut self.allocated_ram_nsc,
        };

        // Disable NSC region, if allocated.
        if nsc_num < allocated_nsc.len() && allocated_nsc[nsc_num] {
            allocated_nsc[nsc_num] = false;

            // XXX: need to find a cleaner way to get these regs.
            let size_reg = match region_type {
                RegionType::Flash => match nsc_num {
                    0 => &self.registers.flashnsc_0_size,
                    1 => &self.registers.flashnsc_1_size,
                    _ => unreachable!(),
                },
                RegionType::Ram => match nsc_num {
                    0 => &self.registers.ramnsc_0_size,
                    1 => &self.registers.ramnsc_1_size,
                    _ => unreachable!(),
                },
            };

            // Disable the region.
            size_reg.write(Size::SIZE::Disabled);
        }
    }

    /// Returns whether or not the given peripheral is present.
    pub fn peripheral_present(&self, periph_id: u32) -> bool {
        let periph_id = periph_id as usize;

        if periph_id >= NUM_PERIPHIDS {
            return false;
        }

        match self.registers.periphid_perm[periph_id].read_as_enum(PeriphIdPerm::PRESENT) {
            Some(PeriphIdPerm::PRESENT::Value::NotPresent) => false,
            Some(PeriphIdPerm::PRESENT::Value::Present) => true,
            None => unreachable!(),
        }
    }

    /// Gets the secure mapping of a peripheral.
    pub fn get_periph_sec(&self, periph_id: u32) -> Result<SecureMapping, Error> {
        // Make sure peripheral is present.
        if !self.peripheral_present(periph_id) {
            return Err(Error::PeripheralNotPresent);
        }

        let periph_id = periph_id as usize;
        let secure_mapping = match self.registers.periphid_perm[periph_id]
            .read_as_enum(PeriphIdPerm::SECUREMAPPING)
        {
            Some(PeriphIdPerm::SECUREMAPPING::Value::NonSecure) => SecureMapping::NonSecure,
            Some(PeriphIdPerm::SECUREMAPPING::Value::Secure) => SecureMapping::Secure,
            Some(PeriphIdPerm::SECUREMAPPING::Value::UserSelectable) => {
                SecureMapping::UserSelectable
            }
            Some(PeriphIdPerm::SECUREMAPPING::Value::Split) => SecureMapping::Split,
            None => unreachable!(),
        };

        Ok(secure_mapping)
    }

    /// Sets the secure mapping of a split or user selectable peripheral. Fails if the security
    /// mapping cannot be changed.
    pub fn set_periph_sec(&self, periph_id: u32, secure: bool) -> Result<(), Error> {
        // Make sure peripheral is present.
        if !self.peripheral_present(periph_id) {
            return Err(Error::PeripheralNotPresent);
        }

        // Make sure secure mapping is modifiable.
        let periph_id = periph_id as usize;
        match self.registers.periphid_perm[periph_id].read_as_enum(PeriphIdPerm::SECUREMAPPING) {
            Some(PeriphIdPerm::SECUREMAPPING::Value::UserSelectable)
            | Some(PeriphIdPerm::SECUREMAPPING::Value::Split) => {
                let secattr = match secure {
                    true => PeriphIdPerm::SECATTR::Secure,
                    false => PeriphIdPerm::SECATTR::NonSecure,
                };
                self.registers.periphid_perm[periph_id].write(secattr);

                Ok(())
            }
            _ => Err(Error::SecureMappingFixed),
        }
    }

    /// Returns whether or not a peripheral's DMA security is configurable.
    pub fn periph_dma_is_configurable(&self, periph_id: u32) -> bool {
        // Make sure peripheral is present.
        if !self.peripheral_present(periph_id) {
            return false;
        }

        // DMA security is only configurable if its type is Separate Attribute.
        let periph_id = periph_id as usize;
        match self.registers.periphid_perm[periph_id].read_as_enum(PeriphIdPerm::DMA) {
            Some(PeriphIdPerm::DMA::Value::SeparateAttribute) => true,
            _ => false,
        }
    }

    /// Sets a peripheral's DMA security, if allowed.
    pub fn set_periph_dma_sec(&self, periph_id: u32, secure: bool) -> Result<(), Error> {
        // Make sure DMA security mapping is configurable.
        if !self.periph_dma_is_configurable(periph_id) {
            return Err(Error::SecureMappingFixed);
        }

        // Set DMA security.
        let periph_id = periph_id as usize;
        let dmasec = match secure {
            true => PeriphIdPerm::DMASEC::Secure,
            false => PeriphIdPerm::DMASEC::NonSecure,
        };
        self.registers.periphid_perm[periph_id].write(dmasec);
        Ok(())
    }
}
