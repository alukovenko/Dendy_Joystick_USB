# Project Makefile

# Binary base name
BINARY = dendy_joystick_usb

# Target MCU
DEVICE = stm32f103c8t6

# Source files
SOURCES = src/main.c

# libopencm3 directory
OPENCM3_DIR = libopencm3

# Include the master build rules
include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

# Flashing configuration
OOCD_INTERFACE = stlink-v2
OOCD_TARGET = stm32f1x

flash: $(BINARY).elf
	openocd -f interface/$(OOCD_INTERFACE).cfg -f target/$(OOCD_TARGET).cfg \
	-c "program $(BINARY).elf verify reset exit"
