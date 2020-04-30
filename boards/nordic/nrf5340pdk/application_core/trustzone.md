# Arm TrustZone on the Nordic nRF5340
Some quick thoughts on what the nRF5340 SPU offers and what a Tock trait for SPU/TrustZone support might look like.

### Contents
* [Background](#background)
* [Proposal](#proposal)
* [Implementation](#implementation)
* [Future Work](#future-work)


## Background
--------------------------------------------------------------------------------

### SPU vs. TrustZone
TrustZone divides flash and RAM into secure and non-secure regions.  This is managed by the SAU (Secure Attribution Unit) or IDAU (Implementation Defined Attribution Unit).  The SAU is provided by Arm while the IDAU may be optionally implemented by the chip vendor.

The SPU (System Protection Unit) is Nordic's implementation of the IDAU and runs in place of the SAU.  In addition to supporting secure/non-secure regions of flash and RAM, it extends TrustZone to restrict peripheral, GPIO pin, and DPPI channel access to secure or non-secure code.

### SPU capabilities we probably care about
* Secure flash regions
  * 64 regions, 16 KiB each (fixed)
    * Early versions of the nRF5340-PDK have 32 regions that are 32 KiB each (see errata #19)
  * Up to two regions may have NSC (Non-Secure Callable) subregions
* Secure RAM regions
  * 64 regions, 8 KiB each (fixed)
  * Up to two regions may have NSC subregions
* Peripheral security
  * User-selectable and split security peripherals can be marked as secure or non-secure by the kernel
  * Some peripherals are only available in secure or non-secure mode
    * Attempting to configure the security of these peripherals should fail
  * Peripherals containing a DMA master may also be able to mark the DMA master as secure or non-secure (separately from the rest of the peripheral)
* GPIO pin access control
  * Can mark GPIO pins as secure so that only secure code can read/write
* External domain access control
  * Restrict access to external domains
  * Currently the only external domain is the network core
* Lock SPU configuration
  * Can independently lock all of the configuration registers until the next reset

#### Other SPU capabilities we probably do not care about (right now)
* DPPI access control
  * Restrict access to DPPI channels which can be used to link different peripheral's tasks/events to each other
  * Tock does not currently use this


## Proposal
--------------------------------------------------------------------------------
Since the SPU offers additional functionality that Arm TrustZone doesn't offer by default, we can offer a generic TrustZone trait and a supplementary SPU trait.

### Traits
``` rust
trait TrustZone {
  /// Unique ID for a secure region.
  type RegionId;
  /// Generic error type.
  type Error;


  /// Create a contiguous secure region.
  pub fn create_secure_region(
    &self,
    region_addr: usize,
    region_size: usize,
  ) -> Result<RegionId, Error>;

  /// Add an NSC subregion to an existing secure region. May fail if no more
  /// NSC subregions may be allocated.
  pub fn create_nsc_region(
    &self,
    region: RegionId,
    nsc_addr: usize,
    nsc_size: usize,
  ) -> Result<(), Error>;
}


trait SPU: TrustZone {
  /// ID type for chip peripherals.
  type PeripheralId;
  /// ID type for external domains.
  type ExtdomainId;


  /// Whether or not SPU can set a peripheral's security configuration.
  pub fn can_set_periph_sec(&self, peripheral: PeripheralId) -> bool;

  /// Mark peripheral as secure or non-secure. May fail if peripheral security
  /// cannot be changed.
  pub fn set_periph_sec(
    &self,
    peripheral: PeripheralId,
    secure: bool,
  ) -> Result<(), Error>;

  /// Get if peripheral is marked secure.
  pub fn get_periph_sec(&self, peripheral: PeripheralId) -> bool;

  /// Whether or not SPU can set a peripheral's DMA security configuration.
  pub fn can_set_dma_sec(&self, peripheral: PeripheralId) -> bool;

  /// Mark a peripheral's DMA master as secure or non-secure. May fail if DMA
  /// security cannot be changed.
  pub fn set_dma_sec(
    &self,
    peripheral: PeripheralId,
    secure: bool,
  ) -> Result<(), Error>;

  /// Get if peripheral's DMA master is marked secure.
  pub fn get_dma_sec(&self, peripheral: PeripheralId) -> bool;


  /// Set a GPIO pin's security.
  pub fn set_gpio_sec(&self, pin: gpio::Pin, secure: bool);

  /// Get a GPIO pin's security.
  pub fn get_gpio_sec(&self, pin: gpio::Pin) -> bool;


  /// Whether or not SPU can set an external domain's security.
  pub fn can_set_extdomain_sec(&self, extdomain: ExtdomainId) -> bool;

  /// Set an external domain's security. May fail if security cannot be
  /// changed.
  pub fn set_extdomain_sec(
    &self,
    extdomain: ExtdomainId,
    secure: bool,
  ) -> Result<(), Error>;

  /// Get an external domain's security.
  pub fn get_extdomain_sec(&self, extdomain: ExtdomainId) -> bool;


  /// Commit SPU configuration by locking all writable registers.
  pub fn commit(&self);
}
```

We probably don't want to be able to change the TrustZone configuration at runtime, so configuration should happen at the start of the reset handler and be committed immediately.

**Questions:**
* Are there any registers that shouldn't be locked when committing?

### Considerations for secure flash and RAM regions
Since flash and RAM have a fixed number of fixed-size regions, there should probably be a new linker section for secure code/data that ensures it complies to secure region boundaries.  Similarly, the linker should also enforce that NSC regions are a valid size and placed at the end of a secure region.  Additionally, we'll have to make sure that non-secure callable functions are placed in NSC regions and have the appropriate `SG` (Secure Gateway) instructions and that calls to non-secure functions from secure regions have the appropriate `BLXNS` instructions.

In addition to being marked as secure, regions can be marked read/write/execute.  This should probably be handled by the MPU instead of our trait.

**Questions:**
* Do we care which specific regions are marked secure?  Or should we just give a desired size for a secure region and let the code/linker figure out where it goes?


## Implementation
--------------------------------------------------------------------------------

TrustZone support is currently partially implemented for the nRF5340.  This includes an SPU driver, modified linker script, and additional TrustZone setup in the board's reset handler.

### SPU driver
The current SPU driver is a quick and basic implementation that doesn't quite follow the proposed TrustZone/SPU traits.  However, it exposes all of the registers and most of the functionality offered by the SPU.  It can get and set the security permissions of flash and RAM regions, GPIO pins, the network core, peripherals, and peripheral DMA masters; create and remove NSC regions; and commit the SPU configuration.

### Linker script
The linker script has also been modified to add a new `.secure` region to the board image.  The `.secure` region is aligned to secure flash region boundaries and contains a `.secure.nsc` section for NSC trampolines that is aligned to the end of the `.secure` section.  Secure code goes into the `.secure.text` section before the NSC section. `_ssecure`, `_esecure`, `_snsc`, and `_ensc` symbols are defined to mark the start and end of the secure and NSC regions, respectivelyl.

### TrustZone setup
The nRF5340-PDK application core also contains additional TrustZone setup code that executes at the start of the reset handler.  The TrustZone setup is secure code that creates the secure and non-secure regions before returning to the rest of the reset handler, which continues executing as non-secure code.

The setup code first iterates through all of the flash and RAM regions, marking them as secure if they are within the `_ssecure` and `_esecure` symbols and non-secure otherwise.  Similarly, the NSC regions in flash and RAM are also configured.  Finally, the peripherals and GPIO pins on the device are marked as secure or non-secure.

### Secure code
The TrustZone setup and other secure code is placed in `src/trustzone.rs`.  Secure code must be marked with the `#[inline(never)]` and `#[link_section = ".secure.text"]` annotations in order to place it within the secure region of the binary.  All secure functions should be annotated with `#[inline(never)]` in order to prevent them from being inlined in non-secure code.

Non-secure callable code must be an `extern "C" fn` marked with the `#[inline(never)]`, `#[link_section = ".secure.text"]`, `#[no_mangle]`, and `#[cmse_nonsecure_entry]` annotations, which places it in the secure region of the binary and generates the appropriate NSC epilogue and other required instructions.  However, this does not generate an `sg` instruction for the function or place it inside of the NSC region.  A secure gateway trampoline must be written in assembly and placed in the `.secure.nsc` section in order to make the function callable, as follows:

``` asm
.section .secure.nsc, "ax"
.global nsc_trampoline
nsc_trampoline:
  sg
  b.w nsc_function
```

Which generates a trampoline called `nsc_trampoline` for a non-secure callable function named `nsc_function`.  Non-secure code must call `nsc_trampoline` instead of `nsc_function`, since only the trampoline is callable from non-secure code.

Note that the TrustZone setup function should have the `#[cmse_nonsecure_entry]` annotation since it returns to non-secure code and needs the NSC epilogue.  However, it does not need a secure gateway trampoline since it will be called by secure code.


## Future Work
--------------------------------------------------------------------------------

Currently, the TrustZone implementation for the nRF5340-PDK does not work correctly.  Code that is supposed to be marked non-secure appears to retain secure privileges.  For example, non-secure code is incorrectly able to call secure functions and write to secure GPIO pins.

It isn't exactly clear why the TrustZone implementation does not work.  The initial implementation only marked the RAM regions that the secure code is mapped to as secure.  It's likely that the flash regions that the secure code is read from must also be marked as secure.  However, this requires that the secure code is placed within the boundaries of secure flash regions, which would probably also require that the secure code be separated into its own binary.  It seems like most other TrustZone implementations require separate secure and non-secure binaries that are combined to form the final image.

Once the TrustZone implementation works, the `TrustZone` and `SPU` traits should be finalized and implemented properly.  It would probably also be nice to write macros that generate the proper boilerplate for secure and NSC functions.  Finally, we would like to write an example application that showcases the TrustZone functionality, such as a software implementation of the [Atmel GLOC](http://ww1.microchip.com/downloads/en/Appnotes/Atmel-42317-Glue-Logic-Controller-GLOC_ApplicationNote_AT07911.pdf) that runs as secure code while the rest of the OS runs as non-secure code.
