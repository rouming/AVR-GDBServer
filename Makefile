#
# Makefile for AVR ATMega16
# Good example of makefiles for AVR
# can be taken from
# http://www.obdev.at/products/vusb/index.html
# e.g. this: examples/hid-mouse/firmware/Makefile
#
CC=avr-gcc
CXX=avr-g++
MMCU=atmega16
ROM_SZ=16384
RAM_SZ=1024

CLK=16000000UL

all:
	avr-gcc -Wall -Os -g -gdwarf-2 -std=gnu99 -mmcu=atmega16 -DF_CPU=16000000UL -o gdb.elf gdb.c main.c -Wl,--section-start=.nrww=0x1fa0

# Flash burning
# to avoid sudo place udev rule for USBASP as /etc/udev/rules.d/usbasp.rules:
# SUBSYSTEMS=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="05dc", GROUP="users", MODE="0666"
flash: all
	avr-objcopy -j .text -j .data -O ihex gdb.elf gdb.hex
	avrdude -c usbasp -p m16 -u -U flash:w:gdb.hex
