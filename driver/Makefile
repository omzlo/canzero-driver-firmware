TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O1 -g
#CFLAGS += -DUSE_FASTER_CAN
ASFLAGS = -g 

# object files

OBJS=  $(STARTUP) main.o
OBJS+= system.o usart.o can.o spi_slave.o systick.o chip_options.o gpio.o nocan.o 

# include common make file

include $(TEMPLATEROOT)/Makefile.common


