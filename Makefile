BINARY = bin
OBJS = main.o

# To disable, run "make ENABLE_SEMIHOSTING=0"
ENABLE_SEMIHOSTING ?= 0

ifeq ($(ENABLE_SEMIHOSTING), 1)
LDFLAGS += --specs=rdimon.specs
LDLIBS += -lrdimon
DEFS += -DENABLE_SEMIHOSTING=1
endif

LDSCRIPT = STM32F103C8T6.ld

include Makefile.include
