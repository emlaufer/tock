//! Implementation of the secure attribution unit for the Cortex-M33.

use kernel;
use kernel::utilities::registers::interfaces::Writeable;
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

/// SAU Registers described in section 4.5 of
/// <https://developer.arm.com/documentation/100235/0004/the-cortex-m33-peripherals/security-attribution-and-memory-protection/security-attribution-unit>
#[repr(C)]
pub struct SauRegisters {
    /// The control register:
    ///   * Enables the SAU (bit 0)
    ///   * Enables marking memory as all secure or non-secure when disabled (bit 1)
    pub ctrl: ReadWrite<u32, Control::Register>,

    /// Indicates how many regions the SAU supports
    pub sau_type: ReadOnly<u32, Type::Register>,

    /// Selects the region number (zero-indexed) referenced by the region base
    /// address and region attribute and size registers.
    pub rnr: ReadWrite<u32, RegionNumber::Register>,

    /// Defines the base address of the currently selected MPU region. Also defines shareability,
    /// access permissions, and execute permissions.
    pub rbar: ReadWrite<u32, RegionBaseAddress::Register>,

    /// Defines the region limit address of the selected MPU region. Also includes attribute index
    /// and region enable bit.
    pub rlar: ReadWrite<u32, RegionLimitAddress::Register>,

    /// Provides information on secure memory related faults.
    pub sfsr: ReadWrite<u32, SecureFaultStatus::Register>,

    /// Provides the address of the memory location that cause a secure memory fault.
    pub sfar: ReadWrite<u32, SecureFaultAddress::Register>,
}

register_bitfields![u32,
    Control [
        /// Controls the default security level of all memory when the SAU is disabled.
        /// When using an IDAU, this should be set to 1.
        ALLNS OFFSET(1) NUMBITS(1) [
            AllSecure = 0,
            AllNonSecure = 1
        ],
        /// Enables the SAU
        ENABLE OFFSET(0) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ]
    ],

    Type [
        /// The number of data regions supported. If this field reads-as-zero the
        /// processor does not implement an SAU, and it should be disabled.
        SREGION OFFSET(8) NUMBITS(8) [],
    ],

    RegionNumber [
        /// Region indicating the SAU region referenced by the SAU_RBAR and
        /// SAU_RASR registers. Range 0-7 corresponding to the SAU regions.
        REGION OFFSET(0) NUMBITS(8) []
    ],

    RegionBaseAddress [
        /// Base address of the currently selected SAU region.
        BADDR OFFSET(5) NUMBITS(27) [],
    ],

    RegionLimitAddress [
        /// Limit address of the currently selected SAU region.
        LADDR OFFSET(5) NUMBITS(27) [],
        /// Controls whether Non-secure code is allowed to execute a SG instruction in this memory.
        NSC OFFSET(1) NUMBITS(1) [
            NotNSC = 0,
            NSC = 1
        ],
        ///Enables the currently selected SAU region.
        EN OFFSET(0) NUMBITS(1) [
            Enable = 0,
            Disable = 1
        ]
    ],

    SecureFaultStatus [
        /// Lazy state error flag. Whether an error occurred during a lazy state activation or
        /// deactivation.
        LSERR OFFSET(7) NUMBITS(1) [],

        /// Whether or not the SFAR address is valid.
        SFARVALID OFFSET(6) NUMBITS(1) [
            NotValid = 0,
            Valid = 1
        ],

        /// Lazy state preservation error flag. Indicates a SAU or IDAU violation occurred during
        /// the lazy preservation of floating-point state.
        LSPERR OFFSET(5) NUMBITS(1) [],

        /// Invalid transition flag. Flag indicating an exception was raised due to an invalid
        /// domain crossing from Secure to Non-Secure memory.
        INVTRAN OFFSET(4) NUMBITS(1) [],

        /// Attribution unit violation flag. Indicates an attempt was made to access part of Secure
        /// memory with NS-Req for a transaction set to Non-secure.
        AUVIOL OFFSET(3) NUMBITS(1) [],

        /// Invalid exception return flag. Can be caused by EXC_RETURN.DCS being set to 0 when
        /// returning from a Non-Secure exception, or by EXC_RETURN.ES being set to 1 when
        /// returning from a Non-Secure exception
        INVER OFFSET(2) NUMBITS(1) [],

        /// Invalid integrity signature flag. Set if the integrity signature in an exception stack
        /// frame is found to be invalid during unstacking.
        INVIS OFFSET(1) NUMBITS(1) [],

        /// Invalid entry point. Set if a function call from Non-secure memory or exception targets
        /// a non-SG instruction in Secure state. Also set if the target address is an SG
        /// instruction but there is no matching secure region in the SAU/IDAU.
        INVEP OFFSET(1) NUMBITS(1) [],
    ],

    SecureFaultAddress [
        /// The address of the memory location that caused a security fault.
        ADDRESS OFFSET(0) NUMBITS(32) [],
    ]
];

const SAU_BASE_ADDRESS: StaticRef<SauRegisters> =
    unsafe { StaticRef::new(0xE000EDD0 as *const SauRegisters) };

// TODO: Better name?
pub enum DefaultMode {
    NonSecure,
    Secure,
}

/// State related to the real physical SAU.
///
/// There should only be one instantiation of this object as it represents
/// real hardware.
pub struct SAU {
    /// MMIO reference to SAU registers.
    registers: StaticRef<SauRegisters>,
}

impl SAU {
    pub const unsafe fn new() -> SAU {
        SAU {
            registers: SAU_BASE_ADDRESS,
        }
    }

    pub fn enable_sau(&self) {
        // TODO: should this check the type to ensure the SAU exists?
        self.registers.ctrl.write(Control::ENABLE::Enable);
    }

    pub fn disable_sau(&self, mode: DefaultMode) {
        let all_ns = match mode {
            DefaultMode::NonSecure => Control::ALLNS::AllNonSecure,
            DefaultMode::Secure => Control::ALLNS::AllSecure,
        };
        self.registers.ctrl.write(Control::ENABLE::Disable + all_ns);
    }

    // TODO: fill in remainin SAU methods...
}
