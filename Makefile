# µBL.  make BOARD=f303-node | make all-boards | make test | make sizes | make regcheck
# A board kept outside boards/ (in the application's repository): BOARD names it, BOARD_DIR points
# at its board.h and board.mk.
BOARD ?= f303-node
BOARD_DIR ?= boards/$(BOARD)
TOOLCHAIN ?= $(wildcard $(CURDIR)/../toolchains/arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi/bin)
CROSS ?= $(if $(TOOLCHAIN),$(TOOLCHAIN)/,)arm-none-eabi-
CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE := $(CROSS)size
PYTHON ?= python3
BUILD ?= build/$(BOARD)

-include $(BOARD_DIR)/board.mk
CAN_FD ?= 0
FRAME := $(if $(filter 1,$(CAN_FD)),fd,classic)
CPU_f0 := cortex-m0
CPU_f3 := cortex-m4
CPU_f4 := cortex-m4
CPU_g4 := cortex-m4
CPU_h7 := cortex-m7
CPU := $(CPU_$(FAMILY))

INCLUDES := -I$(BOARD_DIR) -Isrc -Isrc/hal/stm32 -Isrc/dronecan/$(FRAME)
LTO ?= -flto
CFLAGS := -mcpu=$(CPU) -mthumb -mfloat-abi=soft -Os -g $(LTO) -std=c11 -ffreestanding -fno-common \
	-ffunction-sections -fdata-sections -fno-tree-loop-distribute-patterns -fstack-usage \
	-Wall -Wextra -Wshadow -Wundef -Werror -MMD -MP $(INCLUDES) $(EXTRA_CFLAGS) $(if $(CALLGRAPH),-fcallgraph-info=su)
LDFLAGS := -mcpu=$(CPU) -mthumb -Os $(LTO) -nostdlib -nostartfiles -Wl,--gc-sections -Wl,--no-warn-rwx-segments \
	-Wl,-Map=$(BUILD)/mubl.map -T $(BUILD)/mubl.ld

SRCS := src/main.c src/update.c src/image.c src/handoff.c src/crc.c src/util.c src/uart_mux.c \
	src/proto_dronecan.c src/proto_serial.c src/proto_srxl2.c \
	src/hal/stm32/system.c src/hal/stm32/clock.c src/hal/stm32/gpio.c src/hal/stm32/flash.c \
	src/hal/stm32/bxcan.c src/hal/stm32/fdcan.c src/hal/stm32/uart.c src/hal/stm32/divide.c src/hal/stm32/crc_hw.c \
	src/hal/usb/usb_cdc.c src/hal/usb/usb_fs.c src/hal/usb/usb_otg.c \
	src/dronecan/$(FRAME)/dronecan.c
OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SRCS))

.PHONY: all clean all-boards test test-fast sizes regcheck

all: $(BUILD)/mubl.bin $(BUILD)/mubl.hex
	@$(SIZE) $(BUILD)/mubl.elf

$(BUILD)/mubl.ld: ld/mubl.ld.S $(BOARD_DIR)/board.h src/mubl_config.h src/mubl_defs.h
	@mkdir -p $(@D)
	@$(CC) -E -P -x c -I$(BOARD_DIR) -Isrc $(EXTRA_CFLAGS) $< -o $@

# Each object depends on the headers it includes (the compiler's .d files: board.h, the selected
# frame's generated dronecan.h, ...); before the first build there are none and everything compiles.
$(BUILD)/%.o: %.c
	@mkdir -p $(@D)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

-include $(OBJS:.o=.d)

$(BUILD)/mubl.elf: $(OBJS) $(BUILD)/mubl.ld
	@echo "LD $@"
	@$(CC) $(LDFLAGS) $(OBJS) -lgcc -o $@

$(BUILD)/mubl.bin: $(BUILD)/mubl.elf
	@$(OBJCOPY) -O binary $< $@

$(BUILD)/mubl.hex: $(BUILD)/mubl.elf
	@$(OBJCOPY) -O ihex $< $@

BOARDS := $(patsubst boards/%/board.mk,%,$(wildcard boards/*/board.mk))
all-boards:
	@for b in $(BOARDS); do $(MAKE) --no-print-directory BOARD=$$b || exit 1; done

# Host simulation tests (Python 3, a C compiler; tests/README.md lists what each needs).
test:
	cd tests && $(PYTHON) -m unittest discover -v

sizes:
	$(PYTHON) tools/sizes.py

regcheck:
	$(PYTHON) tests/regcheck/run.py --cc $(CC)

clean:
	rm -rf build
