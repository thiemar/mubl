# Test application for the h735g-dk bootloader: make [APP_VERSION=n] -> build/app.apj
ROOT := ../../..
TOOLCHAIN ?= $(wildcard $(ROOT)/../toolchains/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi/bin)
CROSS ?= $(if $(TOOLCHAIN),$(TOOLCHAIN)/,)arm-none-eabi-
APP_VERSION ?= 1
GIT_HASH := 0x$(shell git -C $(ROOT) rev-parse --short=8 HEAD)
BUILD := build/v$(APP_VERSION)

all: $(BUILD)/app.apj

$(BUILD)/app.elf: app.c app.ld $(ROOT)/app/mubl_app.h
	@mkdir -p $(BUILD)
	$(CROSS)gcc -mcpu=cortex-m7 -mthumb -Os -g -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns -nostdlib -Wall -Wextra -Werror \
		-I$(ROOT)/app -DAPP_VERSION=$(APP_VERSION) -DGIT_HASH=$(GIT_HASH) -T app.ld app.c -lgcc -o $@

$(BUILD)/app.bin: $(BUILD)/app.elf
	$(CROSS)objcopy -O binary $< $@
	python3 $(ROOT)/tools/mubl_image.py stamp $@

$(BUILD)/app.apj: $(BUILD)/app.bin
	python3 $(ROOT)/tools/mubl_image.py apj $< --board-id 7352 -o $@

clean:
	rm -rf build
