PROJECT = dendy_joystick_usb
BUILD_DIR = bin/

CFILES = src/main.c

DEVICE = stm32f103c8t6
OOCD_INTERFACE = stlink-v2
OOCD_TARGET = stm32f1x

VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR = libopencm3/
OPENCM3_LIB = opencm3_stm32f1
OPENCM3_DEFS = -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

bin: $(BUILD_DIR)/$(PROJECT).elf
	$(Q)$(OBJCOPY) -O binary $(BUILD_DIR)/$(PROJECT).elf $(BUILD_DIR)/$(PROJECT).bin

dendy_joystick_usb.flash:
	st-flash write $(BUILD_DIR)/$(PROJECT).bin 0x8000000

all: bin flash
	@echo "Build and flash completed."
