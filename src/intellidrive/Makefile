# Project Details # -----------------------------------------------------------

MCU = atmega328p
F_CPU = 8000000UL
BAUD = 9600UL
LIBDIR = ../lib
LOCLIBDIR = ./lib

# Programmer Defaults # -------------------------------------------------------

PROGRAMMER_TYPE = avrispv2
PROGRAMMER_ARGS = -P /dev/ttyACM0

# Programmer Locations # ------------------------------------------------------

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRSIZE = avr-size
AVRDUDE = avrdude

# Compilation Details # --------------------------------------------------------

# File locations
TARGET = $(lastword $(subst /, ,$(CURDIR)))
SOURCES = $(wildcard *.c $(LOCLIBDIR)/*.c $(LIBDIR)/*.c)
OBJECTS = $(SOURCES:.c=.o)
HEADERS = $(SOURCES:.c=.h)

# Flags
CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. -I$(LOCLIBDIR) -I$(LIBDIR)
#CFLAGS = -Os -g -std=gnu99 -Wall
CFLAGS = -O0 -ggdb -std=gnu99 -Wall
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -Wl,-Map,$(TARGET).map
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -lm
TARGET_ARCH = -mmcu=$(MCU)

# Rules
%.o: %.c $(HEADERS) Makefile
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c -o $@ $<

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

.PHONY: all size clean flash

all: $(TARGET).hex

debug:
	@echo
	@echo "Source files: " $(SOURCES)
	@echo "MCU, F_CPU, BAUD: " $(MCU), $(F_CPU), $(BAUD)
	@echo

help:
	@echo
	@echo "all: 		generates hex from .c file"
	@echo "debug: 		displays sources and general settings used in project"
	@echo "disassemble:	generates .lst file for debugging"
	@echo "size:		generates .elf file and display size information"
	@echo "clean:		removes all generated files"
	@echo "flash:		generates hex and then flashes to MCU"
	@echo

disassemble: $(TARGET).lst

size: $(TARGET).elf
	$(AVRSIZE) -C --mcu=$(MCU) $(TARGET).elf

clean:
	rm -f *.elf *.hex *.obj *.o *.d *.eep *.lst *.lss *.sym *.map *~ *.eeprom; \
	cd ./lib; \
	rm -f *.o; \
	cd ../../lib/; \
	rm -f *.o

flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER_TYPE) -p $(MCU) $(PROGRAMMER_ARGS) -U flash:w:$<

# -----------------------------------------------------------------------------
