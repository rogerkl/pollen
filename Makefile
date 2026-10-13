# Pollen: a granular processor for Electrosmith patch.init, based on Beads

TARGET = pollen

# Paths to the sibling checkouts.
LIBDAISY_DIR = ../libDaisy
EURORACK_DIR = ../eurorack
GCC_PATH ?= ../arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin

# BOOT_NONE: everything in the 128K internal flash.
# BOOT_SRAM: the Daisy bootloader copies the app from QSPI into AXI SRAM.
APP_TYPE ?= BOOT_SRAM

ifeq ($(APP_TYPE), BOOT_SRAM)
LDSCRIPT = pollen_sram.lds
else
LDSCRIPT = pollen_flash.lds
endif

CPP_SOURCES = \
	src/pollen.cpp \
	src/buffer_backup.cpp

# Beads and stmlib use .cc, which libDaisy's makefile does not know about.
CC_SOURCES = \
	beads/dsp/delay_clock.cc \
	beads/dsp/delay_controller.cc \
	beads/dsp/mu_law.cc \
	beads/dsp/processor.cc \
	beads/dsp/trigger_generator.cc \
	beads/gain_control.cc \
	beads/resources.cc \
	$(EURORACK_DIR)/stmlib/dsp/atan.cc \
	$(EURORACK_DIR)/stmlib/dsp/units.cc \
	$(EURORACK_DIR)/stmlib/utils/random.cc

# The local copy of beads/ must come before the eurorack checkout.
C_INCLUDES += -I. -Isrc -I$(EURORACK_DIR)

# Extra defines from the command line, e.g. EXTRA_DEFS=-DPOLLEN_ALIAS_BANKS
C_DEFS += $(EXTRA_DEFS)

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

CC_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CC_SOURCES:.cc=.o)))
OBJECTS += $(CC_OBJECTS)
vpath %.cc $(sort $(dir $(CC_SOURCES)))

$(BUILD_DIR)/%.o: %.cc Makefile | $(BUILD_DIR)
	$(CXX) -c $(CPPFLAGS) $(CPP_STANDARD) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(CC_OBJECTS) $(LDSCRIPT)

# Flashing and the USB serial port from WSL, with usbipd-win on the Windows
# side (see README).
#   make flash                 build, jump to the bootloader, flash, restart
#   make monitor [DURATION=n]  show the status lines, until Ctrl-C by default
#   make send CMD=c            send commands: b, 1-4, c
USBIPD ?= /mnt/c/Program Files/usbipd-win/usbipd.exe
USBIPD_BUSID ?= 1-2
WSL_DISTRO ?= $(WSL_DISTRO_NAME)
COM_PORT ?= COM4
DURATION ?= 0
export USBIPD USBIPD_BUSID WSL_DISTRO COM_PORT

.PHONY: flash monitor send

flash: all
	tools/flash.sh

monitor:
	tools/serial.sh read $(DURATION)

send:
	tools/serial.sh send '$(CMD)'
