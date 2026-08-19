# TODO

- **DualShock 4 units tested with the dongle (extend as more are covered):**
  - CUH-ZCT2U — DS4 v2, North American SKU (USB PID 09CC).
  - CUH-ZCT1U — DS4 v1, North American SKU (USB PID 05C4).

- **ROOT CAUSE FOUND (2026-07-14) for all volume weirdness on Linux:** the
  kernel never created a hardware volume control for the dongle. GET_CUR
  recomputed the volume from config on every read instead of returning the
  last SET value, so the kernel's write-then-readback probe
  (check_sticky_volume_control, sound/usb/mixer.c) flagged the mixer as
  "sticky mixer values ... disabling" (visible in dmesg), dropped the volume
  control, and deliberately pinned the device at max volume. The desktop
  slider was pure PipeWire *software* volume the whole time — the firmware's
  dB→byte mapping (incl. DS4_VOLUME_MAX=87) was never exercised, which is why
  no by-ear model ever fit. Fixed in firmware (GET_CUR now returns the last
  SET value; config only seeds the initial value). After reflash, verify
  `pactl list sinks` shows HW_VOLUME_CTRL and dmesg no longer logs the sticky
  warning. All previous by-ear observations below are void and need re-testing
  with hardware volume active.

- ~~The volume does scale within 0% and 5% up from 10% it just gets very
  loud.~~ **Resolved 2026-07-14** by the sticky-volume fix plus the measured
  mapping below. Re-test on macOS to confirm the log-feel is gone there too.

- Add Microphone support or check if working. I will wire the cable to my speaker out for that. Ask me to do it.
- Sometimes devices starts up with 5% even though shortly after it is 100% battery. maybe add a longer period of uncertainty

- ~~Device is detected as "clone"/"fake" by the web tools.~~ **Resolved
  2026-07-14, verified against both sites.** Cause: feature report 0xA3
  (firmware info) came back 45 instead of 49 bytes over USB — the BT
  response is 3 bytes shorter and the CRC stripping accounts for the rest;
  dualshock-tools' clone check (js/controllers/ds4-controller.js getInfo)
  requires exactly 49 bytes plus a successful 0x81 read. Fixed by
  zero-padding 0xA3 to 48+id bytes; both https://dualshock-tools.github.io/
  and https://ds.daidr.me/ now report the device as genuine, and
  hid-playstation no longer logs "expected 49 got 45" at plug-in.

- Make sure that 2 or more of these firmware dongles are working on one system

- Auto turn off device after 15 minutes of non usage, make this confiurable

- use tools/config_tool.py to make configurations accessible

- **config_tool.py doesn't work on Windows for the DS4 (root cause found
  2026-07-15).** The config uses HID report IDs 0xF6–0xF9. The firmware handles
  them, but they're **not declared** in our DS4 HID report descriptor
  (`desc_hid_report_ds4` in `src/usb_descriptors.cpp`, kept byte-identical to a
  real DS4 v2 → feature reports stop at 0xF2). Windows `hidclass.sys` rejects
  GET/SET_FEATURE for any undeclared report ID → bare `read error`. Confirmed:
  declared reports (0x02/0xA3/0x81/0xF2) read fine, 0xF7 fails. **The WebHID
  tool (ds5.awalol.eu.org) hits the same wall** — it works on the DualSense
  because that firmware declares these reports. Works fine on **Linux** (hidraw
  passes raw requests through, no descriptor check).
  - To support Windows: declare 0xF6–0xF9 as HID feature reports (count 63) in
    `desc_hid_report_ds4` + bump `wDescriptorLength`/static_assert, then reflash.
    Costs USB indistinguishability from a real DS4 v2 → put it behind a build
    flag (e.g. `ENABLE_CONFIG_HID`) so the default build stays stealthy.
  - `config_tool.py` already cleaned up for DS4: correct naming/PID (09CC),
    schema v6, `(DS4: no effect)` tags on DualSense-only fields
    (haptics_gain/headset_volume/speaker_gain/audio_buffer_length/
    controller_mode/trigger_reduce), and an honest Windows diagnostic instead of
    `read error`.

- **Polling rates: proper testing of all three modes.** Rates measured
  2026-07-14 via hidraw timestamps after a config + USB reconnect per mode:
  mode 0 → bInterval 4, exactly 250 reports/s (median gap 4.00 ms);
  mode 1 → bInterval 2, exactly 500/s (2.00 ms); mode 2 → bInterval 1,
  ~755/s effective with 1.00 ms median gap (capped by the DS4's ~800 Hz BT
  rate). Remaining: subjective input-latency check in a game per mode, and
  verify audio (speaker + mic) stays glitch-free per mode — especially
  mode 2 alongside isochronous audio. Also check whether mode 2's
  send-only-when-dirty path drops the report rate when the controller idles
  and whether hosts/games mind.

- ~~Polling rate: config SET_REPORT never reaches the firmware.~~ **Resolved
  2026-07-14: does not reproduce against the current firmware.**
  `config_tool.py set polling_rate_mode=2` updates RAM and flash, and after a
  0xF6/0x03 USB reconnect the gamepad endpoints enumerate with bInterval 1
  (1000 Hz). The earlier failure was most likely an older flashed build (the
  dongle has been reflashed since for the volume-mapping commits). Mode 2 is
  now active on the test dongle; revert with
  `tools/config_tool.py set polling_rate_mode=1` plus a replug if problems
  show up.

- ~~Microphone input (headset mic through the TRRS jack).~~ **WORKING as of
  2026-07-14** — first capture verified (EarPods mic → BT → SBC decode → USB
  IN → parecord). Protocol (reverse-engineered, none of it documented):
  - The DS4 only offers the mic when it detects a DC load on the TRRS
    sleeve (real headset). ext byte bit 6 = mic present. A bare line-out
    into the jack is not detected.
  - Enable: output-report header byte 2 bit 2 (0x24 instead of 0x20).
    The controller then sends one mic-audio input report per output report
    it receives — it must be clocked like a PS4 does; firmware sends a
    no-op 0x11 output every 8 ms while the USB mic is open and speaker
    audio isn't already clocking the link (mic bits are also carried on
    the 0x17 speaker reports).
  - Mic audio arrives in input reports 0x12..0x19 (controller picks the
    size; observed 0x13 at 1 frame/report and 0x16 at 4 frames/report):
    state block (same layout as 0x11) + SBC frames found by 0x9C syncword.
    Format: 16 kHz mono, 16 blocks, 8 subbands, loudness, bitpool 29,
    66-byte frames (header 9c 31 1d).
  - Firmware decodes with btstack's bluedroid SBC decoder on core1 and
    upsamples 16k mono → 32k stereo (linear interpolation) for the USB IN
    endpoint.
  Remaining: voice-quality listening test, simultaneous speaker+mic
  (full duplex) test, decide mic_select semantics (builtin vs headset mic —
  the EnableMic field is 3 bits; bit meanings beyond 0x04 unknown), and
  whether the 8 ms keepalive should be replaced by echoing the host's
  output-report cadence.
  **Hardening verified on-device 2026-07-14 evening:** mic capture, full
  duplex (speaker tone + mic simultaneously), mic-after-duplex (decoder
  survives), and gamepad input during mic (~500 reports/s, sane touch data,
  no kernel errors) all pass. New minor item: capture delivers ~2.6 s per
  8 s window — stream start latency ~5 s after opening the source;
  investigate (PipeWire node resume vs. firmware enable handshake).
  Known issues found in the first long-run (fixes implemented and built,
  flash pending):
  - A misaligned frame (bare 0x9C matched inside SBC payload during the
    duplex test) wedged the OI decoder permanently — sbc frames kept being
    consumed but no PCM came out until reboot. Fix: match the full frame
    header 9c 31 1d and reinit the decoder on every mic open.
  - During duplex the controller sent report variants whose state block is
    not 0x11-layout; hid-playstation rejected the forwarded reports
    ("invalid num_touch_reports=213"), freezing gamepad input. Fix:
    touch-count sanity check before forwarding; needs a proper layout
    capture for those report ids later.
  - Otherwise stable: 3 h of continuous mic streaming (1.36M SBC frames)
    without a firmware crash, queues healthy.

- **Mic under-clocks during duplex audio (analyzed 2026-07-15, NOT yet
  fixed).** Sharpens the open keepalive question above. With both the speaker
  (playback) and mic (capture) endpoints open, the keepalive in `audio_loop`
  is gated off by `!spk_active`, so the mic is clocked only by the 16 ms 0x17
  speaker reports — half the 8 ms it needs (~62 vs ~126 frames/s) →
  jitter-buffer underrun → choppy mic; if the speaker endpoint is open but
  silent (no 0x17) the mic can stall outright. The 2026-07-14 "full duplex …
  all pass" run used a continuous tone (0x17 always flowing) and checked
  survival, not mic frame rate, so it would not have caught this. Designed fix
  (not implemented): shared-clock rework — fire a no-op 0x11 whenever ≥8 ms
  have passed since *any* output report (0x17 or keepalive), regardless of
  spk_active. NOT verified on hardware yet (all mic tests so far mic-only).
  Watch for: choppy/dropping mic in voice chat while game audio plays.

- **Duplex touch guard fixed (PR #6), but stays a heuristic.** The
  touch-count sanity check above was reading the wrong byte — payload offset
  33 (the rolling touch-report timestamp) instead of offset 32
  (num_touch_reports) — so during a mic session (only 0x12–0x19 reports,
  never 0x11) it dropped nearly all controller input: the "missing inputs"
  symptom. PR #6 corrects it to offset 32. The count check is still a
  heuristic (drops anything > 4); a future revision/layout that legitimately
  carries a higher count would drop valid input. First suspect for any
  "controller input freezes during mic/voice chat" report. Non-blocking
  follow-ups (from the PR #6 review): (1) belt-and-suspenders — move
  `set_headset()` above the guard so the headset-jack flag still updates on a
  report that fails the sanity check (guards against headset audio routing to
  the built-in speaker if the guard ever fires); (2) `plug_headset` isn't
  cleared on disconnect, so it latches across an unplug (harmless today --
  the synthetic `state_init_data` report pushed on disconnect zeroes the ext
  byte and so clears it). Note (1) matters more now that `audio_follow_jack`
  gates whether the USB audio device is exposed at all: a jack flag stuck on a
  stale value costs a spurious re-enumeration.

- `audio_follow_jack` (USB audio device only while the jack is occupied, on by
  default) verified on hardware 2026-08-19 (DS4 v1 + Pico 2 W, Windows 11 host): the
  audio endpoints and the `MI_00` audio function come and go with the headset,
  Windows switches its default output, and controller input is not interrupted.
  Known cosmetic caveat: hiding/showing the audio function requires a USB
  re-enumeration, so the host re-detects the controller for a moment on each
  jack transition (visible as a Steam reconnect). Inherent to UAC1 -- interfaces
  cannot be added/removed without leaving the bus. If it ever needs to be less
  visible, `REENUM_GAP_MS` (bus-down window, 150 ms) and `JACK_DEBOUNCE_MS`
  (300 ms) in src/usb.cpp are the knobs.

- Dual audio sinks (speaker + headphone jack as two USB audio functions):
  first attempt failed; retry on top of the fixed sequential L2CAP pairing
  path before assuming the audio code was at fault.

- Higher audio quality: captures show joint-stereo bitpool 51 (report 0x18)
  is accepted — nearly double the bitrate of the current bitpool-25 0x17 path.

- ~~Volume curve fine-tuning / perceived loudness curve.~~ **Resolved
  2026-07-14 with measured data** (tools/volume_sweep.py, jack→line-in RMS
  sweep of the raw byte 0..255): the DS4 headphone amp is linear in dB at
  1.0 dB per byte (0.97 measured) from byte ~20 to byte 80 and saturates
  hard at byte 80 (bytes 80..255 identical within 0.01 dB). Firmware now
  maps byte = 80 + dB and advertises −60..0 dB as the feature-unit range.
  End-to-end verification: sweeping the host hardware control 0..60 tracks
  the analog output dB-for-dB (±0.15 dB) over the full range, with mild
  compression only at the last −56/−60 dB steps. All previous by-ear
  observations were taken with the byte pinned at max (sticky-volume bug)
  and are void.
