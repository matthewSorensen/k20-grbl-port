#  Part of Grbl
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#  Copyright (c) 2012 Sungeun K. Jeon
#  Copyright (c) 2014 Matthew D. Sorensen
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

all: grbl.hex

# User configurable firmware setting:
CLOCK = 48000000
CPU =MK20DX256
# For 3.1. For 3.0, MK20DX128

TEENSY_PATH = /home/matthew/498/teensy-toolchain
COMPILER = $(TEENSY_PATH)/hardware/tools/arm-none-eabi/bin

CFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -nostdlib -MMD -DF_CPU=$(CLOCK) -DUSB_SERIAL -Ivendor/ -D__$(CPU)__
LDFLAGS = -Os -Wl,--gc-sections -mcpu=cortex-m4 -mthumb -Tvendor/mk20dx256.ld
LIBS = -lm
CC = $(COMPILER)/arm-none-eabi-gcc
CXX = $(COMPILER)/arm-none-eabi-g++
OBJCOPY = $(COMPILER)/arm-none-eabi-objcopy
SIZE = $(COMPILER)/arm-none-eabi-size


OBJECTS    = main.o motion_control.o gcode.o spindle_control.o coolant_control.o \
             protocol.o stepper.o settings.o planner.o nuts_bolts.o limits.o \
             print.o report.o


VENDOR_C = $(wildcard vendor/*.c)
VENDOR_OBJECTS = $(patsubst %.c,%.o,$(VENDOR_C))

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@


grbl.elf: $(OBJECTS) $(VENDOR_OBJECTS) 
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) $(VENDOR_OBJECTS) $(LIBS) 

-include $(OBJS:.o=.d)


clean:
	rm -f *.o *.d *.elf *.hex vendor/*.o

