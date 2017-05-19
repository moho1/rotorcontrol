CC=avr-gcc
CFLAGS=-O2 -DF_CPU=16000000UL -mmcu=atmega328p

all: led.hex

program: led.hex
	avrdude -p m328p -P /dev/ttyUSB0 -c arduino -b 57600 -F -u -U flash:w:$<

led.hex: led
	avr-objcopy -O ihex -R .eeprom $< $<.hex
	
