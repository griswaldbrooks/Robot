###############################################################################
# Makefile for the project rtos323
###############################################################################

## General Flags
PROJECT = rtos323
MCU = atmega323
TARGET = rtos323.elf
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
LDFLAGS +=  -Wl,-Map=rtos323.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"C:\Robot\rtos323\." -I"C:\Robot\rtos323\include" -I"C:\Robot\rtos323\portable\GCC\ATMega323" 

## Objects that must be built in order to link
OBJECTS = rtos323.o croutine.o list.o queue.o tasks.o heap_1.o port.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) rtos323.hex rtos323.eep rtos323.lss size

## Compile
rtos323.o: ../rtos323.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

croutine.o: ../croutine.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

list.o: ../list.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

queue.o: ../queue.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

tasks.o: ../tasks.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

heap_1.o: ../portable/MemMang/heap_1.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

port.o: ../portable/GCC/ATMega323/port.c
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
	-rm -rf $(OBJECTS) rtos323.elf dep/* rtos323.hex rtos323.eep rtos323.lss rtos323.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

