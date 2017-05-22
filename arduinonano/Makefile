CC=avr-gcc
CFLAGS=-O2 -DF_CPU=16000000UL -mmcu=atmega328p -Wall -Wextra

PROGNAME=rotorctl

all: ${PROGNAME}.hex

program: ${PROGNAME}.hex
	avrdude -p m328p -P /dev/ttyUSB0 -c arduino -b 57600 -F -u -U flash:w:$<

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@

%.elf: %.c %.h
	${CC} ${CFLAGS} -o $@ $<
	
