# File to be included in all makefiles

TARGET=or1k-elf
#-----------------------------------------------------------
AS		= $(TARGET)-as
CC		= $(TARGET)-gcc
LD		= $(TARGET)-ld
OBJCOPY	= $(TARGET)-objcopy
OBJDUMP	= $(TARGET)-objdump
AR		= $(TARGET)-ar
RANLIB	= $(TARGET)-ranlib
SIZE	= $(TARGET)-size
SIM     = $(TARGET)-sim
GDB     = $(TARGET)-gdb

#-----------------------------------------------------------
WARNINGS= -Wshadow -Wpointer-arith -Wcast-align -Wsign-compare \
		-Waggregate-return -Wstrict-prototypes -Wunused

