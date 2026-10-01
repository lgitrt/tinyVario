# tests/Makefile — host-side unit tests for the pure algorithm modules.
#
# Author: Luca Obwegs
#
# These tests link the real firmware source for madgwick.c / kalman_vz.c
# directly (no mocks) and build with the host's native gcc — no ARM
# toolchain required. Usage:
#   make -C tests        # build
#   make -C tests test   # build and run, non-zero exit on any failure

CC       := gcc
INC_DIR  := ../Core/Inc
SRC_DIR  := ../Core/Src
CFLAGS   := -std=c11 -Wall -Wextra -O2 -I. -I$(INC_DIR)
LDLIBS   := -lm

BIN_DIR  := build

TARGETS  := $(BIN_DIR)/test_madgwick $(BIN_DIR)/test_kalman_vz

.PHONY: all test clean

all: $(TARGETS)

$(BIN_DIR):
	-mkdir $(BIN_DIR)

$(BIN_DIR)/test_madgwick: test_madgwick.c $(SRC_DIR)/madgwick.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BIN_DIR)/test_kalman_vz: test_kalman_vz.c $(SRC_DIR)/kalman_vz.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test: all
	$(BIN_DIR)/test_madgwick
	$(BIN_DIR)/test_kalman_vz

clean:
	rm -rf $(BIN_DIR)
