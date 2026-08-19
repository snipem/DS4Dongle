# Local mirror of .github/workflows/build.yml: build the production and
# debug firmware variants.

PICO_SDK_PATH ?= $(CURDIR)/.deps/pico-sdk
GENERATOR     := Ninja

VERSION := $(shell git describe --tags --exact-match 2>/dev/null || git rev-parse --short=7 HEAD)

PYTHON ?= python3
PORT   ?= 8000

PRODUCTION_DIR := build/standard
DEBUG_DIR      := build/serial

.PHONY: all build production debug deploy deploy-debug serve clean distclean

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

## Reboot the attached dongle into BOOTSEL and flash the production build.
## reboot_bootsel.py "failing" is expected if the dongle is already sitting
## in BOOTSEL mass-storage mode (no HID device to send it to) -- ignore it
## and let flash.sh find the drive either way.
deploy: production
	-python3 tools/reboot_bootsel.py
	tools/flash.sh $(PRODUCTION_DIR)/ds4-bridge.uf2 30

## Same, but flashes the debug (serial console) build.
deploy-debug: debug
	-python3 tools/reboot_bootsel.py
	tools/flash.sh $(DEBUG_DIR)/ds4-bridge.uf2 30

## Serve the WebHID config UI (tools/config_web.html) over http://localhost.
## WebHID needs a secure context, so the page has to be served -- opening it as
## a file:// URL leaves navigator.hid undefined. Override the port with
## `make serve PORT=9000`. Ctrl-C to stop.
serve:
	@echo "DS4Dongle config UI: http://localhost:$(PORT)/tools/config_web.html"
	$(PYTHON) -m http.server $(PORT)

clean:
	cmake --build $(PRODUCTION_DIR) --target clean 2>/dev/null || true
	cmake --build $(DEBUG_DIR) --target clean 2>/dev/null || true

distclean:
	rm -rf $(PRODUCTION_DIR) $(DEBUG_DIR)
