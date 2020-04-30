# Makefile for building the tock kernel for the nRF development kit

TARGET=thumbv8m.main-none-eabi
PLATFORM=nrf5340pdk-app

RUSTC_FLAGS ?= \
  -C link-arg=-Tlayout.ld \
  -C linker=rust-lld \
  -C linker-flavor=ld.lld \
  -C relocation-model=dynamic-no-pic \
  -C link-arg=-zmax-page-size=512 \
  -C link-arg=-icf=all \

include ../../../Makefile.common

TOCKLOADER=tockloader
NRFJPROG=nrfjprog

# Where in flash to load the kernel with `tockloader`
KERNEL_ADDRESS=0x00000

# Upload programs over uart with tockloader
ifdef PORT
  TOCKLOADER_GENERAL_FLAGS += --port $(PORT)
endif

TOCKLOADER_JTAG_FLAGS = --jlink --board nrf5340dk

# Upload the kernel over JTAG
.PHONY: flash
flash: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(TOCKLOADER) $(TOCKLOADER_GENERAL_FLAGS) flash --address $(KERNEL_ADDRESS) $(TOCKLOADER_JTAG_FLAGS) $<

# Upload the kernel over serial/bootloader
.PHONY: program
program: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex
	$(NRFJPROG) --program $< --chiperase
	$(NRFJPROG) --pinreset

# Convert .bin to .hex
$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(OBJCOPY) $< -O ihex $@
