LIBNAME	= ../libusb/usbstack
APPNAME = eprom-emulator

# Tool definitions
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld -v
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
RM		= rm

# Tool flags
CFLAGS  = -Wno-unused-variable -I./ -I../libusb -c -W -Wall -O0 -g -DDEBUG -mcpu=arm7tdmi
ASFLAGS = -ahls -mapcs-32 -o crt.o
LFLAGS  =  -nostartfiles
CPFLAGS = -O ihex
ODFLAGS	= -x --syms

EPROMFILE = firmware.bin
LINKFILE	= lpc2148-rom.ld

CSRCS	= startup.c printf.c
OBJS 	= crt.o $(CSRCS:.c=.o)

all: depend eprom-emulator

eprom.h: .FORCE
	@ echo "Building EPROM header eprom.h from $(EPROMFILE)..."
	@ python -c 'import os, sys; s=os.path.getsize("$(EPROMFILE)"); print "Firmware size: %u" % s; sys.exit(not (s == 16384 or s == 32768))'
	@ python -c 'import textwrap; s=open("$(EPROMFILE)", "rb").read().encode("hex").upper(); t="".join(["\\x"+x+y for (x,y) in zip(s[0::2], s[1::2])]) ; print "static const char eprom_bin[] = \\\n\t\"%s\";"%"\" \\\n\t\"".join(textwrap.wrap(t,80))' > eprom.h
	
eprom-emulator: $(OBJS) eprom.h eprom-emulator.o serial_fifo.o armVIC.o $(LIBNAME).a
	@ echo "Building $@..."
	$(CC) -T $(LINKFILE) $(LFLAGS) $^ -o $@.elf -Wl,-Map,$@.map
	$(CP) $(CPFLAGS) $@.elf $@.hex
	$(OD) $(ODFLAGS) $@.elf > $@.dmp

crt.o: crt.s
	@ echo ".assembling"
	$(CC) -c $(AFLAGS) -Wa,-ahlms=crt.lst crt.s -o crt.o

clean:
	rm -f eprom.h *.hex *.elf *.o *.lst *.dmp *.map .depend

# recompile if the Makefile changes
$(OBJS): Makefile

# dependency checking
depend: $(CSRCS)
	$(CC) $(CFLAGS) -MM $^ > .depend || rm -f .depend

# phony targets
.FORCE:
.PHONY: .FORCE clean

-include .depend

