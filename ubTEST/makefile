
# The name of the project
PRG            = ubTest
# Files owned by the user
OBJUSER        = ubTest.o 
# Generated files that are required
OBJREQD        = 
# Generated files that are optional
OBJOPT         = 

# The processor target
MCU_TARGET     = atmega640

# The processor speed
MCU_SPEED	   = 16000000

# The library where WebbotLib is installed
WEBBOT		   = "../../webbotavrclib-1.30"


OPTIMIZE       = -O0

DEFS           = -gdwarf-2 -std=gnu99 -fpack-struct -fshort-enums  -funsigned-char -funsigned-bitfields -I$(WEBBOT)
DEFS 		  +=  -MD -MP -MT $(*F).o -MF $(*F).d


LIBS           = -L$(WEBBOT) -lWebbot-ATmega640  -lm -lc
	

# Define the tools to be used
CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE		   = avr-size
ARCHIVE		   = avr-ar

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall -DF_CPU=${MCU_SPEED} -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

# Default build target
all: $(PRG).elf lst text eeprom size

# How to create the elf file
$(PRG).elf:  $(OBJUSER) $(OBJREQD)
	$(CC) -mmcu=$(MCU_TARGET) $(LDFLAGS) -o $@ $(OBJUSER) $(OBJREQD) $(LIBS)



# dependencies for compiling C files to O files

	
ubTest.o: ubTest.c
		$(CC) $(CFLAGS) $(OPTIMIZE) -c -o $@ ubTest.c

size: $(PRG).elf
	$(SIZE) --format=avr --mcu=${MCU_TARGET} $(PRG).elf	

# Clean the compiled stuff but leave any codegen'd files in place
clean:
	rm -rf *.o *.d lib/*.o lib/*.a $(PRG).elf example.txt  
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

# Clean out everything - including any codegen'd files
cleanall:
	rm -rf *.o *.d lib/*.o lib/*.a lib/*.c $(PRG).elf example.txt hardware.h  
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images
text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images
eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }

EXTRA_CLEAN_FILES       = *.hex *.bin *.srec

	