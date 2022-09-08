# Name: Makefile
# Project: AQ-IRR-1
# Author: Andrey Grigorev
# Creation Date: 2022-02-16
# Tabsize: 4
# Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
# License: GPLv2.

VERSION=V9_0
PROJ=AQIRR1
TARGET_NAME=$(PROJ)_$(VERSION)

DEVICE=attiny45
AVRDUDE = avrdude -c arduino -P /dev/ttyACM0 -p $(DEVICE)
# The two lines above are for "avrdude" and the STK500 programmer connected
# to an USB to serial converter to a Mac running Mac OS X.
# Choose your favorite programmer and interface.

COMPILE = avr-gcc -Wall -Os -Iusbdrv -I. -mmcu=$(DEVICE) -DF_CPU=16500000 -DDEBUG_LEVEL=0
# NEVER compile the final product with debugging! Any debug output will
# distort timing so that the specs can't be met.

OBJECTS = usbdrv/usbdrv.o usbdrv/usbdrvasm.o usbdrv/oddebug.o main.o RC_func/ir_funcs.o

# symbolic targets:
all:	$(TARGET_NAME).hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:$(TARGET_NAME).hex:i -F


# Fuse high byte:
# 0xdd = 1 1 0 1   1 1 0 1
#        ^ ^ ^ ^   ^ \-+-/ 
#        | | | |   |   +------ BODLEVEL 2..0 (brownout trigger level -> 2.7V)
#        | | | |   +---------- EESAVE (preserve EEPROM on Chip Erase -> not preserved)
#        | | | +-------------- WDTON (watchdog timer always on -> disable)
#        | | +---------------- SPIEN (enable serial programming -> enabled)
#        | +------------------ DWEN (debug wire enable)
#        +-------------------- RSTDISBL (disable external reset -> enabled)
#
# Fuse low byte:
# 0xe1 = 1 1 1 0   0 0 0 1
#        ^ ^ \+/   \--+--/
#        | |  |       +------- CKSEL 3..0 (clock selection -> HF PLL)
#        | |  +--------------- SUT 1..0 (BOD enabled, fast rising power)
#        | +------------------ CKOUT (clock output on CKOUT pin -> disabled)
#        +-------------------- CKDIV8 (divide clock by 8 -> don't divide)
fuse:
	$(AVRDUDE) -U hfuse:w:0xdd:m -U lfuse:w:0xe1:m -F

readcal:
	$(AVRDUDE) -U calibration:r:/dev/stdout:i | head -1


clean:
	rm -f $(TARGET_NAME).hex $(TARGET_NAME).lst $(TARGET_NAME).obj $(TARGET_NAME).cof $(TARGET_NAME).list $(TARGET_NAME).map $(TARGET_NAME).eep.hex $(TARGET_NAME).bin *.o usbdrv/*.o RC_func/*.o RC_func/*.s $(TARGET_NAME).s usbdrv/oddebug.s usbdrv/usbdrv.s

# file targets:

$(TARGET_NAME).hex:	$(TARGET_NAME).bin
	rm -f $(TARGET_NAME).hex $(TARGET_NAME).eep.hex
	avr-objcopy -j .text -j .data -O ihex $(TARGET_NAME).bin $(TARGET_NAME).hex
	./checksize $(TARGET_NAME).bin 4096 256
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

$(TARGET_NAME).bin:	$(OBJECTS)
	$(COMPILE) -o $(TARGET_NAME).bin $(OBJECTS)

disasm:	$(TARGET_NAME).bin
	avr-objdump -d $(TARGET_NAME).bin

cpp:
	$(COMPILE) -E main.c
