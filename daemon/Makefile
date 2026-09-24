CC      ?= gcc
DRV     := ../driver
CFLAGS  ?= -O2 -g -Wall -Wextra -Wpedantic -std=c11
CFLAGS  += -Iinclude -I$(DRV)/include
LDLIBS  := -pthread -lrt -lm
BIN     := build

DAEMON_SRC := src/so101d.c src/control.c src/supervisor.c src/calib.c $(DRV)/src/feetech.c
HDRS       := $(wildcard include/*.h) $(DRV)/include/feetech.h

all: $(BIN)/so101d $(BIN)/so101ctl $(BIN)/test_calib

$(BIN):
	mkdir -p $(BIN)

$(BIN)/so101d: $(DAEMON_SRC) $(HDRS) | $(BIN)
	$(CC) $(CFLAGS) -o $@ $(DAEMON_SRC) $(LDLIBS)

$(BIN)/so101ctl: tools/so101ctl.c include/so101_shm.h | $(BIN)
	$(CC) $(CFLAGS) -o $@ tools/so101ctl.c $(LDLIBS)

$(BIN)/test_calib: tests/test_calib.c src/calib.c include/calib.h | $(BIN)
	$(CC) $(CFLAGS) -o $@ tests/test_calib.c src/calib.c -lm

test: $(BIN)/test_calib
	./$(BIN)/test_calib

clean:
	rm -rf $(BIN)

.PHONY: all test clean
