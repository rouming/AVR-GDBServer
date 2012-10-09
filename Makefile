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
