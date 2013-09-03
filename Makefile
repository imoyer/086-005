# GESTALT FIRMWARE MAKEFILE
# REVISED: 3/1/2013
# Ilan Moyer and Nadya Peek
# www.pygestalt.org

#Change filename as appropriate. Note: assumes .cpp
#PROJECT = 086-005a_boot
PROJECT = 086-005a

MCU = atmega328p
FREQ = 18432000	

#ADDRESS = 0x7000
ADDRESS = 0x0000

#LIST ALL CUSTOM DEFINES HERE
#GESTALT_DEFS = -DstandardGestalt -Dbootloader -DsingleStepper -Dgestalt328
GESTALT_DEFS = -DstandardGestalt -DnetworkedGestalt -DsingleStepper -Dgestalt328

GESTALT_DIR = /Users/imoyer/gsArduino


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
