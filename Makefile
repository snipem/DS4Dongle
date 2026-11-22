# Local mirror of .github/workflows/build.yml: build the production and
# debug firmware variants.

PICO_SDK_PATH ?= $(CURDIR)/.deps/pico-sdk
GENERATOR     := Ninja

# Tag on HEAD if there is one, else the short hash. Written without shell
# redirection/`||` so it also works when make runs recipes through cmd.exe
# (GnuWin32 make on Windows), where the old one-liner produced an empty VERSION.
VERSION := $(or $(firstword $(shell git tag --points-at HEAD)),$(shell git rev-parse --short=7 HEAD))

ifeq ($(OS),Windows_NT)
PYTHON ?= python
else
PYTHON ?= python3
endif
PORT   ?= 8000

PRODUCTION_DIR := build/standard
DEBUG_DIR      := build/serial
OPINIONATED_DIR := build/opinionated

# `make flash` target selection: production | debug | opinionated.
# Opinionated is the default because it is the only variant whose reboot and
# version reports work on Windows (vanilla needs the physical BOOTSEL button).
VARIANT       ?= opinionated
FLASH_TIMEOUT ?= 30
# The dongle only re-enumerates once a DualShock 4 connects -- allow time for that.
FLASH_ENUM_TIMEOUT ?= 120
FLASH_DIR_production  := $(PRODUCTION_DIR)
FLASH_DIR_debug       := $(DEBUG_DIR)
FLASH_DIR_opinionated := $(OPINIONATED_DIR)
FLASH_SUFFIX_opinionated := -opinionated

.PHONY: all build production debug opinionated flash deploy deploy-debug deploy-opinionated serve clean distclean

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

## Opinionated firmware (Windows-configurable, 1 kHz + wake-on-PS default)
## -> $(OPINIONATED_DIR)/ds4-bridge.uf2
opinionated:
	cmake -S . -B $(OPINIONATED_DIR) -G $(GENERATOR) \
		-DCMAKE_BUILD_TYPE=Release \
		-DPICO_SDK_PATH=$(PICO_SDK_PATH) \
		-DVERSION=$(VERSION) \
		-DOPINIONATED=ON
	cmake --build $(OPINIONATED_DIR)

## Build $(VARIANT), then run the full BOOTSEL track on the attached dongle:
## reboot into BOOTSEL (or wait for the button), copy the .uf2, confirm the
## bootloader accepted it, wait for re-enumeration (switch the DualShock 4 on
## when prompted -- the dongle only appears on USB with a controller) and check the reported
## firmware version. Fails loudly at any step.
##   make flash                      # opinionated build
##   make flash VARIANT=production
flash: $(VARIANT)
	$(PYTHON) tools/flash.py $(FLASH_DIR_$(VARIANT))/ds4-bridge.uf2 \
		--expect-version $(VERSION)$(FLASH_SUFFIX_$(VARIANT)) \
		--timeout $(FLASH_TIMEOUT) --enum-timeout $(FLASH_ENUM_TIMEOUT)

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

## Same, but flashes the opinionated build.
deploy-opinionated: opinionated
	-python3 tools/reboot_bootsel.py
	tools/flash.sh $(OPINIONATED_DIR)/ds4-bridge.uf2 30

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
	cmake --build $(OPINIONATED_DIR) --target clean 2>/dev/null || true

distclean:
	rm -rf $(PRODUCTION_DIR) $(DEBUG_DIR) $(OPINIONATED_DIR)
