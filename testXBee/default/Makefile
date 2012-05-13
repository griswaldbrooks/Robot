###############################################################################
# Makefile for the project testXBee
###############################################################################

## General Flags
PROJECT = testXBee
MCU = atmega640
TARGET = testXBee.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -Os -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=testXBee.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Robot\testXBee\SoR" -I"C:\Robot\testXBee\." 

## Library Directories
LIBDIRS = -L"C:\Robot\testXBee\SoR" -L"C:\Robot\testXBee\." 

## Libraries
LIBS = -lc -lm 

## Objects that must be built in order to link
OBJECTS = testXBee.o uart4.o buffer.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) testXBee.hex testXBee.eep testXBee.lss size

## Compile
testXBee.o: ../testXBee.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

uart4.o: ../SoR/uart4.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

buffer.o: ../SoR/buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) testXBee.elf dep/* testXBee.hex testXBee.eep testXBee.lss testXBee.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

