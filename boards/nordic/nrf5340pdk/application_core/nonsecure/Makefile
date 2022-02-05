# Makefile for building the tock kernel for the nRF development kit

TARGET=thumbv8m.main-none-eabi
PLATFORM=nrf5340pdk-app-nonsecure
NSC_SYMS=nrf5340dk-app-nsc.syms

RUSTC_FLAGS ?= \
  -C link-arg=-Tlayout.ld \
  -C linker=rust-lld \
  -C linker-flavor=ld.lld \
  -C relocation-model=dynamic-no-pic \
  -C link-arg=-zmax-page-size=512 \
  -C link-arg=-icf=all \

# TODO: make more generic for release vs debug
RUSTC_FLAGS_FOR_BIN ?= -C link-arg=-L$(abspath .) -C link-arg=--just-symbols=$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(NSC_SYMS)

include ../../../../Makefile.common

TOCKLOADER=tockloader
NRFJPROG=nrfjprog

# Where in flash to load the kernel with `tockloader`
KERNEL_ADDRESS=0x00000

# Upload programs over uart with tockloader
ifdef PORT
  TOCKLOADER_GENERAL_FLAGS += --port $(PORT)
endif

TOCKLOADER_JTAG_FLAGS = --jlink --board nrf5340dk

# Default target for installing the kernel.
.PHONY: install
install: flash

# Upload the kernel over JTAG
.PHONY: flash
flash: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(TOCKLOADER) $(TOCKLOADER_GENERAL_FLAGS) flash --address $(KERNEL_ADDRESS) $(TOCKLOADER_JTAG_FLAGS) $<

# nrfjprog --recover screws up normal reset and debugging...use JLink reset instead
.PHONY: recover
recover:
	 echo "r" | JLinkExe -device nRF5340_xxAA_APP -if swd -speed 1000

# Upload the kernel over serial/bootloader
.PHONY: program
program: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex
	#$(NRFJPROG) --recover
	$(NRFJPROG) --program $< --chiperase
	$(NRFJPROG) --reset

# upload the kernel over jlink, halted for debugging
.PHONY: jlink
jlink:
	JLinkExe -device nRF5340_xxAA_APP -if swd -speed 1000

hex: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex

# Convert .bin to .hex
$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(OBJCOPY) $< -O ihex $@
