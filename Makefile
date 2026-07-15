# Local mirror of .github/workflows/build.yml: build the production and
# debug firmware variants.

PICO_SDK_PATH ?= $(CURDIR)/.deps/pico-sdk
GENERATOR     := Ninja

VERSION := $(shell git describe --tags --exact-match 2>/dev/null || git rev-parse --short=7 HEAD)

PRODUCTION_DIR := build/standard
DEBUG_DIR      := build/serial

.PHONY: all build production debug clean distclean

all: build

## Build both firmware variants (default)
build: production debug

## Production firmware -> $(PRODUCTION_DIR)/ds4-bridge.uf2
production:
	cmake -S . -B $(PRODUCTION_DIR) -G $(GENERATOR) \
		-DCMAKE_BUILD_TYPE=Release \
		-DPICO_SDK_PATH=$(PICO_SDK_PATH) \
		-DVERSION=$(VERSION)
	cmake --build $(PRODUCTION_DIR)

## Debug firmware (serial console) -> $(DEBUG_DIR)/ds4-bridge.uf2
debug:
	cmake -S . -B $(DEBUG_DIR) -G $(GENERATOR) \
		-DCMAKE_BUILD_TYPE=Release \
		-DPICO_SDK_PATH=$(PICO_SDK_PATH) \
		-DVERSION=$(VERSION) \
		-DENABLE_SERIAL=ON -DENABLE_VERBOSE=ON
	cmake --build $(DEBUG_DIR)

clean:
	cmake --build $(PRODUCTION_DIR) --target clean 2>/dev/null || true
	cmake --build $(DEBUG_DIR) --target clean 2>/dev/null || true

distclean:
	rm -rf $(PRODUCTION_DIR) $(DEBUG_DIR)
