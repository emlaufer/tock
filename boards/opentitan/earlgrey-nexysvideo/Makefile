# Makefile for building the tock kernel for the OpenTitan platform

DEFAULT_BOARD_CONFIGURATION=fpga_nexysvideo
TARGET=riscv32imc-unknown-none-elf
PLATFORM=earlgrey-nexysvideo
FLASHID=--dev-id="0403:6010"
RISC_PREFIX ?= riscv64-elf
QEMU ?= ../../../tools/qemu/build/qemu-system-riscv32


include ../../Makefile.common

# Pass OpenTitan board configuration option in `BOARD_CONFIGURATION` through
# Cargo `--features`. Please see `Cargo.toml` for available options.
ifneq ($(BOARD_CONFIGURATION),)
	CARGO_FLAGS += --features=$(BOARD_CONFIGURATION)
else
	CARGO_FLAGS += --features=$(DEFAULT_BOARD_CONFIGURATION)
endif

# Default target for installing the kernel.
.PHONY: install
install: flash

qemu: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(call check_defined, OPENTITAN_BOOT_ROM)
	$(QEMU) -M opentitan -kernel $^ -bios $(OPENTITAN_BOOT_ROM) -nographic -serial mon:stdio

qemu-app: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(call check_defined, OPENTITAN_BOOT_ROM)
	$(QEMU) -M opentitan -kernel $^ -bios $(OPENTITAN_BOOT_ROM) -device loader,file=$(APP),addr=0x20030000 -nographic -serial mon:stdio

flash: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(OPENTITAN_TREE)/build-out/sw/host/spiflash/spiflash $(FLASHID) --input=$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin

flash-app: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(RISC_PREFIX)-objcopy --update-section .apps=$(APP) $^ $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM)-app.elf
	$(RISC_PREFIX)-objcopy --output-target=binary $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM)-app.elf $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM)-app.bin
	$(OPENTITAN_TREE)/build-out/sw/host/spiflash/spiflash $(FLASHID) --input=$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM)-app.bin

verilator: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(call check_defined, OPENTITAN_TREE)
	srec_cat $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin \
		--binary --offset 0 --byte-swap 8 --fill 0xff \
		-within $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin \
		-binary -range-pad 8 --output binary.64.vmem --vmem 64
	$(OPENTITAN_TREE)/build/lowrisc_dv_chip_verilator_sim_0.1/sim-verilator/Vchip_sim_tb \
		--meminit=rom,$(OPENTITAN_TREE)/build-out/sw/device/boot_rom/boot_rom_sim_verilator.scr.39.vmem \
		--meminit=flash,./binary.64.vmem \
		--meminit=otp,$(OPENTITAN_TREE)/build-bin/sw/device/otp_img/otp_img_sim_verilator.vmem

test:
ifneq ($(OPENTITAN_TREE),)
	$(error "Running on QEMU, use test-hardware to run on hardware")
endif
	mkdir -p $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/
	$(Q)cp test_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/layout.ld
	$(Q)cp ../../kernel_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/
	$(Q)RUSTFLAGS="$(RUSTC_FLAGS_TOCK)" $(CARGO) test $(CARGO_FLAGS_TOCK_NO_BUILD_STD) $(NO_RUN) --bin $(PLATFORM) --release

test-hardware:
ifeq ($(OPENTITAN_TREE),)
	$(error "Please ensure that OPENTITAN_TREE is set")
endif
	mkdir -p $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/
	$(Q)cp test_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/layout.ld
	$(Q)cp ../../kernel_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/
	$(Q)RUSTFLAGS="$(RUSTC_FLAGS_TOCK)" $(CARGO) test $(CARGO_FLAGS_TOCK_NO_BUILD_STD) $(NO_RUN) --bin $(PLATFORM) --release --features=hardware_tests

test-verilator:
	$(call check_defined, OPENTITAN_TREE)

	mkdir -p $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/
	$(Q)cp test_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/deps/layout.ld
	$(Q)cp ../../kernel_layout.ld $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/

	$(Q)RUSTFLAGS="$(RUSTC_FLAGS_TOCK)" VERILATOR="yes" $(CARGO) test $(CARGO_FLAGS_TOCK_NO_BUILD_STD) $(NO_RUN) --bin $(PLATFORM) --release --features=hardware_tests
