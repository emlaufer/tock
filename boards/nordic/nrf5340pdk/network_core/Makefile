# Makefile for building the tock kernel for the nRF development kit

TARGET=thumbv7em-none-eabi
PLATFORM=nrf5340pdk-net

include ../../../Makefile.common

TOCKLOADER=tockloader
NRFJPROG=nrfjprog

# Where in flash to load the kernel with `tockloader`
KERNEL_ADDRESS=0x1000000

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
	$(NRFJPROG) --coprocessor CP_NETWORK --program $< --chiperase
	$(NRFJPROG) --pinreset

# Convert .bin to .hex
$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(OBJCOPY) $< -O ihex $@
