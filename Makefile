CC=avr-gcc
CPU=atmega16
CFLAGS=-c -g -O3 -Wall -mmcu=$(CPU)
LDFLAGS=--cref -mmcu=$(CPU)

SRC = src
BUILD = build
EXE = a3

SRCS = $(wildcard $(SRC)/*.c)
OBJS = $(subst .c,.o,$(subst $(SRC),$(BUILD), $(SRCS)))

all:	init $(EXE).hex

$(BUILD)/%.o : $(SRC)/%.c
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/$(EXE).elf: $(BUILD)/$(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.hex : $(BUILD)/%.elf
	avr-objcopy -O ihex $< $@

%.flash : %.hex
	avrdude -pm16 -cstk500v2 -P/dev/ttyS0 -Uflash:w:$<

init:
	mkdir -p $(BUILD)

clean:
	rm -f $(SRC)/*~
	rm -rf $(BUILD)/*
	rm -f *.hex
	
