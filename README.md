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
- Opinionated variant: configurable on Windows, 1 kHz by default, and Wi-Fi
  Wake-on-LAN to power the PC on with the PS button (see
  [Firmware variants](#firmware-variants))

## Configuring

Two front-ends for the same HID config reports:

- `tools/config_tool.py` -- CLI (`get`, `set name=value ...`, `fields`).
- `tools/config_web.html` -- a WebHID page for Chrome/Edge. WebHID needs a
  secure context, so serve it rather than opening the file directly:

  ```sh
  make serve          # or: make serve PORT=9000
  # then open http://localhost:8000/tools/config_web.html
  ```

With the **vanilla** firmware both are blocked on **Windows**: the config report
IDs 0xF6-0xF9 are handled by the firmware but are not declared in the HID report
descriptor (which is kept byte-identical to a real DS4 v2), and Windows drops
GET/SET_FEATURE for any undeclared report id. They work on Linux, macOS and
ChromeOS as-is. The **opinionated** firmware declares the reports, so both tools
work on Windows too (see [Firmware variants](#firmware-variants)).

## Firmware variants

| | `ds4-bridge.uf2` (vanilla) | `ds4-bridge-opinionated.uf2` |
|---|---|---|
| USB persona | byte-identical to a real DS4 v2 | DS4 v2 + declared config reports 0xF6-0xFA (distinguishable) |
| Config tools on Windows | no | yes |
| Default polling rate | 250 Hz (stock) | real-time / 1 kHz |
| Wi-Fi Wake-on-LAN | no | yes |

Waveshare RP2350B-Plus-W builds: `ds4-bridge-waveshare.uf2` (vanilla) and
`ds4-bridge-waveshare-opinionated.uf2`.

The default polling rate only applies to a fresh config; a dongle that already
has a saved config keeps its `polling_rate_mode` when you switch variants.

### Wi-Fi Wake-on-LAN (opinionated)

Wakes a PC that is off, hibernating, or asleep without USB remote wakeup:

1. You press PS; the controller connects to the dongle.
2. If there is no USB data connection at that moment (the PC has not
   enumerated the dongle, or the bus is suspended), the dongle immediately joins
   the configured Wi-Fi network and broadcasts a magic packet for the PC's
   network card every second (raw EtherType 0x0842 frame plus a UDP broadcast
   to port 9). If the PC turns out to be on, it enumerates the dongle before the
   Wi-Fi join finishes and Wi-Fi is switched off again without sending anything.
3. It stops the moment the PC enumerates the dongle, or after 5 minutes.

Wi-Fi is fully off (disassociated, WLAN core down) whenever the PC is up, so
Bluetooth has the radio to itself during play. Only a *fresh* controller
connection arms it, so shutting the PC down with the controller still on does
not wake it back up.

Configure it in `config_web.html` (Wake-on-LAN section, with a **Test** button)
or with the CLI:

```sh
python tools/config_tool.py set wol_ssid=MyWifi wol_password=- wol_mac=aa:bb:cc:dd:ee:ff wol_enabled=1
python tools/config_tool.py wol-test   # join now + send 5 packets, shows the result
```

The password is write-only (the firmware never reports it back) but is stored
unencrypted in the dongle's flash. Requirements:

- A **2.4 GHz** network with WPA2-PSK or open (the Pico's radio has no 5 GHz;
  WPA3-only networks will not work).
- The USB port must stay **powered while the PC is off** (often "ErP" off /
  "USB power in S4/S5" on in the BIOS). With no power the dongle can't do anything.
- Wake-on-LAN (magic packet) enabled in the BIOS and on the NIC in Windows,
  and the PC wired to the same LAN/broadcast domain as the Wi-Fi.

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

`-DENABLE_SERIAL=ON -DENABLE_VERBOSE=ON` builds the debug variant;
`-DOPINIONATED=ON` (or `make opinionated`) builds the opinionated variant.

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
