#
#  Generic and Simple Atmel AVR GNU Makefile
#
#  Desinged for the gnu-avr tool chain
#
# 	Features
#		- upload
#		- create exe from library
#		- create assembler listing (.dis)
#
#	Limitations
#		- only C-files supported
#		- no automatic dependency checking (call 'make clean' if any .h files are changed)
#
#	Targets:
#		make
#			create hex file, no upload
#		make upload
#			create and upload hex file
#		make clean
#			delete all generated files
#
#  Note:
#  	Display list make database: make -p -f/dev/null | less

#================================================
# Project Information
TARGETNAME = tiny_led_light
MCU:=attiny85
F_CPU:=8000000
SRC = $(shell ls *.c 2>/dev/null)

#================================================
# System/Environment Information
AVRTOOLSPATH:=/usr/bin/
# Type: "avrdude -c ?" to get a full listing.
AVRDUDE_PROGRAMMER := avrispmkii
# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT := usb

#================================================
# Main part of the Makefile starts here. Usually no changes are needed.

# Internal Variable Names
AVRDUDE_FLASH = -U flash:w:$(TARGETNAME).hex
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -v -v
LIBNAME:=$(TARGETNAME).a
ELFNAME:=$(TARGETNAME).elf
HEXNAME:=$(TARGETNAME).hex
DISNAME:=$(TARGETNAME).dis
OBJ := $(SRC:.c=.o)
# Replace standard build tools by avr tools
CC = $(AVRTOOLSPATH)avr-gcc
AR  = @$(AVRTOOLSPATH)avr-ar
# AVR GNU Tools
OBJCOPY:=$(AVRTOOLSPATH)avr-objcopy
OBJDUMP:=$(AVRTOOLSPATH)avr-objdump
SIZE:=$(AVRTOOLSPATH)avr-size
AVRDUDE = avrdude
# C flags
COMMON_FLAGS = -DF_CPU=$(F_CPU) -mmcu=$(MCU) $(DOGDEFS)
COMMON_FLAGS += -g -Os -Wall -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
COMMON_FLAGS += -I. -I$(DOGDIR)
COMMON_FLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections
COMMON_FLAGS += -Wl,--relax
COMMON_FLAGS += -mcall-prologues
CFLAGS = $(COMMON_FLAGS) -std=gnu99 -Wstrict-prototypes  

# Additional Suffixes
.SUFFIXES: .elf .hex .dis

# Targets
.PHONY: all
all: $(DISNAME) $(HEXNAME)
	$(SIZE) $(ELFNAME)

.PHONY: upload
upload: $(DISNAME) $(HEXNAME)
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_FLASH)
	$(SIZE) $(ELFNAME)

.PHONY: zip
zip:
	zip $(TARGETNAME).zip $(SRC) Makefile

.PHONY: clean
clean:
	$(RM) $(HEXNAME) $(ELFNAME) $(LIBNAME) $(DISNAME)

# implicit rules
.elf.hex:
	@$(OBJCOPY) -O ihex -R .eeprom $< $@

# explicit rules
$(ELFNAME): $(LIBNAME)($(OBJ)) 
	$(LINK.o) $(COMMON_FLAGS) $(LIBNAME) $(LOADLIBES) $(LDLIBS) -o $@

$(DISNAME): $(ELFNAME)
	$(OBJDUMP) -S $< > $@

	
