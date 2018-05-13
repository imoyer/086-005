# GESTALT FIRMWARE MAKEFILE
# REVISED: 3/1/2013
# REVISED: 10/18/2013: added -networkedGestalt lib, now compiles bootloader correctly
# Ilan Moyer and Nadya Peek
# www.pygestalt.org

#Change filename as appropriate. Note: assumes .cpp

GESTALT_DIR = ../gestalt/gsArduino

MCU = atmega328p
FREQ = 18432000	

#bootloader
PROJECT = 086-005a_boot
ADDRESS = 0x7000 
GESTALT_DEFS = -DstandardGestalt -Dbootloader -DsingleStepper -Dgestalt328 -DnetworkedGestalt

#application program, uncomment if you want to use this instead.
#PROJECT = 086-005a
#ADDRESS = 0x0000
#GESTALT_DEFS = -DstandardGestalt -DnetworkedGestalt -DsingleStepper -Dgestalt328

#----INNER WORKINGS BEGIN HERE----
GESTALT_FILE = $(GESTALT_DIR)/gestalt.cpp
LDSECTION = --section-start=.text=$(ADDRESS)
SOURCES = $(PROJECT).cpp $(GESTALT_FILE)

CFLAGS = -g -Wall -Os -mmcu=$(MCU) -DF_CPU=$(FREQ) -I$(GESTALT_DIR) $(GESTALT_DEFS)
LDFLAGS = -Wl,$(LDSECTION)

all: $(PROJECT).hex clean

$(PROJECT).o: $(SOURCES)
	avr-g++ $(CFLAGS) -c -Wall $(SOURCES)

$(PROJECT).elf: $(PROJECT).o
	avr-g++ $(CFLAGS) $(LDFLAGS) gestalt.o -o $@ $^

$(PROJECT).hex: $(PROJECT).elf
	avr-objcopy -j .text -j .data -O ihex $< $@

clean:
	rm -rf *.o *.elf

####
# load program code
####
# note: if you only have the 328 instead of the 328p, just compile for 328 
# and load with the -p m328 flag. You'll need to add 328 to your avrdude.

program-avrisp2-fuses:
	avrdude -c avrisp2 -P usb -p m328p -U efuse:w:0x5:m -F	
	#note that only first 3 bits can be set
	avrdude -c avrisp2 -P usb -p m328p -U hfuse:w:0xD8:m -F
	avrdude -c avrisp2 -P usb -p m328p -U lfuse:w:0xEF:m -F

program-avrisp2:
#	uncomment to program bootloader
#	avrdude -e -c avrisp2 -P usb -p m328p -U flash:w:086-005a_boot.hex
#	uncomment to program application
	avrdude -e -c avrisp2 -P usb -p m328p -U flash:w:086-005a.hex

program-usbtiny:
#	uncomment to program bootloader
#	avrdude -e -c usbtiny -P usb -p m328p -U flash:w:086-005a_boot.hex
#	uncomment to program application
	avrdude -e -c usbtiny -P usb -p m328p -U flash:w:086-005a.hex

program-usbtiny-fuses:
	avrdude -c usbtiny -P usb -p m328p -U efuse:w:0x5:m -F	
	#note that only first 3 bits can be set
	avrdude -c usbtiny -P usb -p m328p -U hfuse:w:0xD8:m -F
	avrdude -c usbtiny -P usb -p m328p -U lfuse:w:0xEF:m -F



