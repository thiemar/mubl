# Test application with a USB console, for an H7 bootloader board: it brings up USB with µBL's own
# driver (product "mubl test app") and answers keys on it.
#   make [BOARD=tbs-lucid-h7|h735g-dk] [APP_VERSION=n]  -> build/<board>-v<n>/app.{bin,apj}
ROOT := ../../..
BOARD ?= tbs-lucid-h7
TOOLCHAIN ?= $(wildcard $(ROOT)/../toolchains/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi/bin)
CROSS ?= $(if $(TOOLCHAIN),$(TOOLCHAIN)/,)arm-none-eabi-
APP_VERSION ?= 1
GIT_HASH := 0x$(shell git -C $(ROOT) rev-parse --short=8 HEAD)
BUILD := build/$(BOARD)-v$(APP_VERSION)
DEFS := -DMUBL_TEST_BOARD='"$(ROOT)/boards/$(BOARD)/board.h"' -DAPP_VERSION=$(APP_VERSION) -DGIT_HASH=$(GIT_HASH)
INCLUDES := -I. -I$(ROOT)/src -I$(ROOT)/src/hal/stm32 -I$(ROOT)/src/dronecan/classic -I$(ROOT)/app
SRCS := app.c $(ROOT)/src/hal/usb/usb_cdc.c $(ROOT)/src/hal/usb/usb_otg.c $(ROOT)/src/hal/usb/usb_fs.c \
	$(ROOT)/src/hal/stm32/gpio.c $(ROOT)/src/hal/stm32/clock.c $(ROOT)/src/util.c
CFLAGS := -mcpu=cortex-m7 -mthumb -Os -g -std=c11 -ffreestanding -fno-tree-loop-distribute-patterns \
	-ffunction-sections -fdata-sections -Wall -Wextra -Werror $(INCLUDES) $(DEFS)
BOARD_ID = $(shell printf 'MUBL_BOARD_ID\n' | $(CROSS)gcc -E -P -x c $(INCLUDES) $(DEFS) -include board.h \
	-include mubl_config.h - | tail -1)

all: $(BUILD)/app.apj

$(BUILD)/app.ld: app.ld.S board.h
	@mkdir -p $(BUILD)
	$(CROSS)gcc -E -P -x c $(INCLUDES) $(DEFS) $< -o $@

$(BUILD)/app.elf: $(SRCS) $(BUILD)/app.ld board.h $(ROOT)/app/mubl_app.h $(ROOT)/boards/$(BOARD)/board.h
	$(CROSS)gcc $(CFLAGS) -nostdlib -Wl,--gc-sections -T $(BUILD)/app.ld $(SRCS) -lgcc -o $@

$(BUILD)/app.bin: $(BUILD)/app.elf
	$(CROSS)objcopy -O binary $< $@
	python3 $(ROOT)/tools/mubl_image.py stamp $@

$(BUILD)/app.apj: $(BUILD)/app.bin
	python3 $(ROOT)/tools/mubl_image.py apj $< --board-id $(BOARD_ID) -o $@

clean:
	rm -rf build
