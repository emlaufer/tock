
TARGET=thumbv8m.main-none-eabi
PLATFORM=nrf5340pdk-app

# TODO make this more general...
SECURE_HEX=../../../../target/$(TARGET)/release/$(PLATFORM)-secure.hex
NONSECURE_HEX=../../../../target/$(TARGET)/release/$(PLATFORM)-nonsecure.hex

NRFJPROG=nrfjprog

# Upload the kernel over serial/bootloader
.PHONY: program
program: 
	make -C secure hex
	# TODO: this causes a cargo rebuild...just put in elf compilation step later
	make -C secure nsc_symtab
	make -C nonsecure hex
	#$(NRFJPROG) --recover
	$(NRFJPROG) --program $(SECURE_HEX) --chiperase
	$(NRFJPROG) --program $(NONSECURE_HEX)
	$(NRFJPROG) --reset

# nrfjprog --recover screws up normal reset and debugging...use JLink reset instead
.PHONY: recover
recover:
	 echo "r" | JLinkExe -device nRF5340_xxAA_APP -if swd -speed 1000
