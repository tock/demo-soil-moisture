# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Copyright Tock Contributors 2022.

# Makefile for building the Tock kernel.

TOCK_ARCH=cortex-m4
TARGET=thumbv7em-none-eabi
PLATFORM=makepython-soil-moisture

include ../Makefile.common

# Default target for installing the kernel.
.PHONY: install
install: program

# Upload the kernel using tockloader and the tock bootloader
.PHONY: program
program: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	tockloader flash --address 0x10000 $<

.PHONY: flash-bootloader
flash-bootloader:
	curl -L --output /tmp/makepython-nrf52840-bootloader_v1.1.3.bin https://github.com/tock/tock-bootloader/releases/download/v1.1.3/makepython-nrf52840-bootloader_v1.1.3.bin
	tockloader flash --address 0 /tmp/makepython-nrf52840-bootloader_v1.1.3.bin
	rm /tmp/makepython-nrf52840-bootloader_v1.1.3.bin
