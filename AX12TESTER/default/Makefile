###############################################################################
# Makefile for the project AX12TESTER
###############################################################################

## General Flags
PROJECT = AX12TESTER
MCU = atmega2560
TARGET = AX12TESTER.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -DF_CPU=16000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=AX12TESTER.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Robot\AX12TESTER\." -I"C:\Robot\AX12TESTER\AX12" -I"C:\Robot\AX12TESTER\SoR" 

## Libraries
LIBS = -lc -lm 

## Objects that must be built in order to link
OBJECTS = AX12TESTER.o ax12.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) AX12TESTER.hex AX12TESTER.eep AX12TESTER.lss size

## Compile
AX12TESTER.o: ../AX12TESTER.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ax12.o: ../AX12/ax12.c
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
	-rm -rf $(OBJECTS) AX12TESTER.elf dep/* AX12TESTER.hex AX12TESTER.eep AX12TESTER.lss AX12TESTER.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

