CC      ?= gcc
CFLAGS  ?= -O2 -g -Wall -Wextra -Wpedantic -std=c11 -Iinclude
BIN     := build

all: $(BIN)/fts_tool $(BIN)/jitter_test $(BIN)/test_protocol

$(BIN):
	mkdir -p $(BIN)

$(BIN)/fts_tool: tools/fts_tool.c src/feetech.c include/feetech.h | $(BIN)
	$(CC) $(CFLAGS) -o $@ tools/fts_tool.c src/feetech.c

$(BIN)/jitter_test: tools/jitter_test.c src/feetech.c include/feetech.h | $(BIN)
	$(CC) $(CFLAGS) -pthread -o $@ tools/jitter_test.c src/feetech.c

$(BIN)/test_protocol: tests/test_protocol.c src/feetech.c include/feetech.h | $(BIN)
	$(CC) $(CFLAGS) -o $@ tests/test_protocol.c src/feetech.c

test: $(BIN)/test_protocol
	./$(BIN)/test_protocol

clean:
	rm -rf $(BIN)

.PHONY: all test clean
