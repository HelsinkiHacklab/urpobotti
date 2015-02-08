CC         = gcc
CFLAGS     = -g -Wall -std=gnu99 -fstack-protector
LDFLAGS    =
PROGRAMS   =

PROGRAMS += pixy_test
PIXY_TEST_SRC = pixy_test.c


CFLAGS_WITH_PTHREAD = $(CFLAGS) -pthread
CFLAGS_WITH_ZMQ = $(shell pkg-config --cflags libzmq) $(CFLAGS) 
LIBS_WITH_ZMQ = $(shell pkg-config --libs libzmq) $(LIBS) 

CFLAGS	+= -g $(shell pkg-config --cflags libusb) -lrt
LDFLAGS	+= -g
LIBS 	+= $(shell  pkg-config --libs libusb) -lpixyusb

all: $(PROGRAMS)


pixy_test: $(PIXY_TEST_SRC)
	$(CC) $(PIXY_TEST_SRC) $(CFLAGS) $(LDFLAGS) -o $@ $(LIBS)

clean:
	@rm -rf $(PROGRAMS)


