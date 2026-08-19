# DS4Dongle — Pico 2 W DualShock 4 Bridge

Firmware for the Raspberry Pi Pico 2 W that hosts a DualShock 4 over
Bluetooth Classic and presents it to the PC as a **wired DualShock 4 v2**
(054C:09CC) — including audio to the controller's speaker and headphone jack.

Adapted from [awalol/DS5Dongle](https://github.com/awalol/DS5Dongle), which
does the same for the DualSense. If you have a DualSense, use DS5Dongle
directly.

See [Releases](https://github.com/snipem/DS4Dongle/releases) for the images.

> **Status: pre-release.** Works on the author's hardware (DS4 v1 + Pico 2 W,
> Linux host). Microphone input is not implemented yet. Expect rough edges.

## Features

- All Buttons work
- Touchpad works (one or two touch points and clicks)
- Gyroscope and Accelerometer works
- LED
- Charging detection

- Wired DS4 v2 USB persona with the real HID report descriptor: gamepad,
  motion sensors, touchpad, rumble, lightbar
- Calibration passthrough (BT report 0x05 translated to USB report 0x02)
- Audio out to the controller speaker and headphone jack: USB audio at the
  DS4's native 32 kHz stereo, SBC-encoded on-device (no resampling); routing
  auto-switches when a headset is plugged into the jack
- Headset-style audio device (`audio_follow_jack`, on by default): like a PS4,
  the dongle only presents a USB audio device while something is plugged into
  the controller's 3.5 mm jack, so Windows and friends switch their default
  output to it on plug and back off on unplug. Two consequences: while the jack
  is empty there is no audio device at all (the built-in speaker is unreachable
  too, unless `speaker_select=1`), and showing/hiding it requires a USB
  re-enumeration, so the host briefly re-detects the controller on each jack
  transition (Steam shows a reconnect; input is not interrupted). Set
  `audio_follow_jack=0` for a permanently visible audio device
- Volume/mute from the host mapped to the controller
- Pairing and controller management via the BOOTSEL button, persistent
  pairings and blacklist in flash
- Configurable over HID feature reports (`tools/config_tool.py`, or
  `tools/config_web.html` in Chrome/Edge): polling rate, audio routing,
  jack-following audio device, inactivity timeout, wake-on-PS, and more

## Configuring

Two front-ends for the same HID config reports:

- `tools/config_tool.py` -- CLI (`get`, `set name=value ...`, `fields`).
- `tools/config_web.html` -- a WebHID page for Chrome/Edge. WebHID needs a
  secure context, so serve it rather than opening the file directly:

  ```sh
  make serve          # or: make serve PORT=9000
  # then open http://localhost:8000/tools/config_web.html
  ```

Both are blocked on **Windows**: the config report IDs 0xF6-0xF9 are handled by
the firmware but are not declared in the HID report descriptor (which is kept
byte-identical to a real DS4 v2), and Windows drops GET/SET_FEATURE for any
undeclared report id. They work on Linux, macOS and ChromeOS as-is.

## Flashing

1. Hold BOOTSEL while plugging the Pico 2 W in; it mounts as a USB drive.
2. Copy `ds4-bridge.uf2` onto it.

Grab pre-built firmware from the Releases page (`ds4-bridge.uf2`; the
`-debug` variant adds a USB serial console for troubleshooting and changes
the USB persona — don't use it for play).

## Pairing

1. With the controller off, hold **Share**, then **PS**, until the lightbar
   double-flashes rapidly.
2. Short-press the Pico's **BOOTSEL button**. The LED blinks while searching
   and turns solid when connected; the lightbar flashes gold once.
3. Afterwards a plain PS press reconnects. The dongle only appears on USB
   while a controller is connected.

BOOTSEL while running: **click** = pair/switch controller, **double-click** =
reboot, **triple-click** = reboot into the bootloader for flashing,
**hold ~1.5 s** = forget all controllers.

## Building

Requires `arm-none-eabi-gcc`, CMake, Ninja, and pico-sdk 2.3.0 with TinyUSB
pinned to 0.21.0:

```sh
git clone --depth 1 --branch 2.3.0 https://github.com/raspberrypi/pico-sdk
git -C pico-sdk submodule update --init --recursive
git -C pico-sdk/lib/tinyusb fetch --depth 1 origin refs/tags/0.21.0:refs/tags/0.21.0
git -C pico-sdk/lib/tinyusb checkout 0.21.0

cmake -S . -B build -G Ninja -DPICO_SDK_PATH=$PWD/pico-sdk
cmake --build build
# → build/ds4-bridge.uf2
```

`-DENABLE_SERIAL=ON -DENABLE_VERBOSE=ON` builds the debug variant.

## Debugging

The debug firmware (`ds4-bridge-debug.uf2`, or a `-DENABLE_SERIAL=ON` build)
differs from the production build in three ways: it adds a USB CDC serial
console, it stays on the USB bus from boot (the production build hides USB
until a controller is connected), and the watchdog is disabled so a fault
prints instead of silently rebooting.

To use it:

1. Flash the debug UF2 (see [Flashing](#flashing)).
2. Open the serial console — the port appears as soon as the Pico boots:

   ```sh
   # Linux (any of these)
   tio /dev/ttyACM0
   picocom -b 115200 /dev/ttyACM0
   # Windows: PuTTY on the new COM port, 115200 baud
   ```

3. Reproduce the problem (pair, connect, play audio). Log lines are prefixed
   by subsystem: `[HCI]` Bluetooth link events (inquiry, connect, auth,
   encryption, disconnect reasons), `[L2CAP]` HID channel setup and traffic,
   `[BT]` button actions and pairing state, `[BLACKLIST]` pairing blacklist,
   `[AUDIO]`/`[Audio]` USB audio and SBC pipeline, `[CMD]`/`[Config]` config
   reports, `[USBHID]` report forwarding.

What healthy output looks like: pairing runs `Gamepad found` →
`ACL connected` → `Authentication complete` → `Encryption change ... enabled=1`
→ `HID Control opened` → `HID Interrupt opened` → `Connected DS4 Controller`.
A `Disconnected reason=0x13` right after `HID Control opened` means the
controller aborted the handshake; `reason=0x08` is a supervision timeout
(range/battery). If the controller connects and immediately drops with the
production build but works in debug, suspect the watchdog (a stall >1 s
reboots the dongle).

Notes: the debug build's USB persona differs (extra CDC interface, audio
visible before a controller connects), so drivers and games may treat it
differently — use it for diagnosis only. While USB audio is streaming, the
console fills with send-FIFO warnings during (re)connects; pause or suspend
the dongle's audio sink to get a clean pairing trace.

## Resources

Built on the work of others:

- [awalol/DS5Dongle](https://github.com/awalol/DS5Dongle) — the codebase
  this is forked from (BT host, USB bridge, config system, RAM relocation)
- [psdevwiki DS4-BT](https://www.psdevwiki.com/ps4/DS4-BT) — report layouts,
  audio report 0x17, CRC scheme
- [sensepost dual-pod-shock](https://github.com/sensepost/dual-pod-shock) —
  DS4 Bluetooth audio research
- [GP2040-CE](https://github.com/OpenStickCommunity/GP2040-CE) — the DS4 v2
  HID report descriptor
- [usedbytes/picow_ds4](https://github.com/usedbytes/picow_ds4) — prior art
  for DS4-on-Pico
- Raspberry Pi pico-sdk, BlueKitchen BTstack, TinyUSB

Developed with the assistance of Claude (Fable 5), verified on real hardware.

## License

MIT, same as DS5Dongle. See [LICENSE](LICENSE).
