# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected. We recommend that you leave it undefined and
#                add settings like this to your ~/.avrduderc file:
#                   default_programmer = "stk500v2"
#                   default_serial = "avrdoper"
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega328p
CLOCK      = 16000000
#PROGRAMMER = -c avrisp -b 19200 -P /dev/tty.usbmodemfa131
PROGRAMMER = -c usbtiny
#OBJECTS    = main.o timer0.o I2C.o ad5142.o  irmp.o usart.o lcd.o ds1307.o sd_raw.o fat.o partition.o byteordering.o
OBJECTS    = main.o I2C.o usart.o mcp23017.o
#OBJECTS    = main.o timer0.o I2C.o ad5142.o  irmp.o
#FUSES      = -U lfuse:w:0x62:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m # 1MHz
#FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m # 8MHz
FUSES      = -U lfuse:w:0xce:m -U hfuse:w:0xd9:m -U efuse:w:0x07:m # Ext 8+MHz
CSTANDARD = -std=gnu99
CFLAGS += $(CSTANDARD)


# ATMega8 fuse bits used above (fuse bits for other devices are different!):
# Example for 8 MHz internal oscillator
# Fuse high byte:
# 0xd9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (clock option, depends on oscillator type)
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ WDTON (if set to 0, watchdog is always on)
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
# Fuse low byte:
# 0x24 = 0 0 1 0   0 1 0 0
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (8M internal RC)
#        | |  +--------------- SUT 1..0 (slowly rising power)
#        | +------------------ BODEN (if 0, brown-out detector is enabled)
#        +-------------------- BODLEVEL (if 0: 4V, if 1: 2.7V)
#
# For computing fuse byte values for other devices and options see
# the fuse bit calculator at http://www.engbedded.com/fusecalc/


# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude -B 1 $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc $(CFLAGS) -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)


flash:
	$(AVRDUDE) -U flash:w:./Release/101FM_data_logger.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

dump:
	$(AVRDUDE) -F -U flash:r:./Backups/101FM_data_logger_$(shell date -u +'%y.%m.%d_%H:%M:%S').hex:i
