all : clean WS2812B.hex burn WS2812B.lst
CC = avr-gcc

OBJS = WS2812B_main.o WS2812B_usb.o
SRCS = WS2812B_main.c WS2812B_usb.c

PROCESSOR=atmega32u4
PROGRAMCODE=m32u4
CFLAGS = -Wall -Os -mmcu=$(PROCESSOR) -DF_CPU=16000000UL -I. -Iusbdrv
ASFLAGS = $(CFLAGS) -x assembler-with-cpp

WS2812B.elf : $(OBJS)
	avr-gcc -I -mmcu=$(PROCESSOR) $(CFLAGS) -Wl,-Map,WS2812B.map -o $@ $^ -L /usr/lib64/binutils/avr/2.19.1

WS2812B.hex : WS2812B.elf
	avr-objcopy -j .text -j .data -O ihex WS2812B.elf WS2812B.hex

WS2812B.lst : $(SRCS)
	avr-gcc -c -g -Wa,-a,-ad $(CFLAGS) $^ > $@

burn : WS2812B.hex
	avrdude -c usbasp -p $(PROGRAMCODE) -U flash:w:WS2812B.hex -F

readfuses :
	avrdude -c usbasp -p $(PROGRAMCODE) -U hfuse:r:high.txt:b -U lfuse:r:low.txt:b

burnfuses :
	avrdude -c usbasp -p $(PROGRAMCODE) -U lfuse:w:0xEE:m -U hfuse:w:0xD9:m -U efuse:w:0xCC:m
#Setup clock / Disable hardware booter - we want the SPI Programmer only!

clean :
	rm -f *~ high.txt low.txt WS2812B.hex WS2812B.map WS2812B.elf $(OBJS) *.o WS2812B.lst

