#!/usr/bin/env python3
"""
Read and modify the DS4Dongle (DualShock 4 v2 bridge) configuration over USB HID,
without reflashing.

Protocol (see src/cmd.cpp / src/config.h):
  GET feature report 0xF7 -> raw Config_body bytes
  GET feature report 0xF8 -> firmware version string
  SET feature report 0xF6:
      funcid 0x01 + body   -> update config in RAM (firmware clamps invalid values)
      funcid 0x02          -> persist config to flash
      funcid 0x03          -> reconnect the USB device

Config_body is a packed struct; this tool derives the binary layout from FIELDS.

PLATFORM NOTE (Windows): the config report IDs 0xF6-0xF9 are handled by the
firmware but are deliberately NOT declared in the DS4 HID report descriptor
(it is kept byte-identical to a real DS4 v2, whose feature reports stop at 0xF2).
Windows' HID class driver rejects GET/SET_FEATURE for any report ID absent from
the descriptor, so this tool cannot reach the config on Windows with stock
firmware -- every command fails with "read error". It works on Linux, where
hidraw passes the raw request through regardless of the descriptor. See
open_device()/read_config() for the diagnostic. To use it on Windows the
firmware must declare 0xF6-0xF9 as HID feature reports (and be reflashed).

Requires: pip install hidapi

Examples (Linux):
  python config_tool.py get
  python config_tool.py set speaker_volume=90 enable_wake=1
  python config_tool.py set inactive_time=10 --no-save
  python config_tool.py fields
"""
import platform
import argparse
import struct
import sys


def _load_hid():
    try:
        import hid
    except ImportError:
        sys.exit("Missing dependency. Install with:  pip install hidapi")
    return hid


VID = 0x054C
# DS4Dongle always enumerates as DualShock 4 v2 (0x09CC). The DualSense PIDs are
# kept only so the tool can also talk to the upstream ds5dongle firmware.
PIDS = (0x09CC, 0x0CE6, 0x0DF2)  # DS4 v2 (DS4Dongle), DualSense, DualSense Edge
HID_USAGE_PAGE_GENERIC_DESKTOP = 0x01
HID_USAGE_GAMEPAD = 0x05

REPORT_SET = 0xF6        # SET_REPORT: write/save config
REPORT_GET_CONFIG = 0xF7  # GET_REPORT: read Config_body
REPORT_GET_VERSION = 0xF8  # GET_REPORT: firmware version string

FUNC_UPDATE = 0x01       # update config in RAM
FUNC_SAVE = 0x02         # persist to flash
FUNC_RECONNECT = 0x03    # reconnect tinyusb device

SET_DATA_LEN = 63        # data bytes after the report id (descriptor report count 0x3F)
FEATURE_REPORT_LEN = SET_DATA_LEN + 1  # report id + descriptor report count

CONFIG_VERSION = 6       # src/config.cpp CONFIG_VERSION (display only)

# struct.pack/unpack codes per field kind. "resN" is N reserved bytes: a slot
# the firmware's Config_body (src/config.h) still defines for a DualSense-only
# setting the DS4 does not use. This is a DS4-only tool, so those slots carry no
# name or meaning here -- they exist purely to keep the packed layout aligned
# with the firmware. Reserved bytes unpack to a bytes object and are written
# back verbatim; they are never shown or set.
KIND_TO_CODE = {"u8": "B", "res1": "1s", "res2": "2s", "res4": "4s"}
RESERVED_KINDS = {"res1", "res2", "res4"}

# FIELDS is the single source of truth for the packed Config_body layout
# (src/config.h). To add/remove/reorder a field, edit ONLY this table -- the
# binary format (STRUCT_FMT) is derived from the 'kind' column below.
# name, kind, validator(value)->bool, help. Order and sizes MUST match
# Config_body. The res* slots stand in for fields the DS4 firmware ignores; do
# not repurpose them without matching the firmware struct and bumping
# CONFIG_VERSION.
FIELDS = [
    ("config_version",     "u8",    lambda v: True,              "config schema version (read-only, managed by firmware)"),
    ("_reserved_a",        "res4",  None,                        ""),  # firmware haptics_gain (float)
    ("speaker_volume",     "u8",    lambda v: 0 <= v <= 127,     "[0, 127] (seeds the initial USB speaker volume)"),
    ("_reserved_b",        "res2",  None,                        ""),  # firmware headset_volume, speaker_gain
    ("inactive_time",      "u8",    lambda v: 0 <= v <= 60,      "[0, 60] minutes (0 disable)"),
    ("disable_pico_led",   "u8",    lambda v: v in (0, 1),       "0/1"),
    ("polling_rate_mode",  "u8",    lambda v: v in (0, 1, 2),    "0:250Hz 1:500Hz 2:real-time"),
    ("_reserved_c",        "res2",  None,                        ""),  # firmware audio_buffer_length, controller_mode
    ("enable_usb_sn",      "u8",    lambda v: v in (0, 1),       "0/1 (USB serial number)"),
    ("ps_shortcut_enabled","u8",    lambda v: v in (0, 1),       "0/1 (Xbox Game Bar via HID keyboard)"),
    ("mic_select",         "u8",    lambda v: v in (0, 1, 2, 3), "0:auto 1:builtin 2:headphone 3:disable"),
    ("speaker_select",     "u8",    lambda v: v in (0, 1, 2, 3), "0:auto 1:builtin 2:headphone 3:disable"),
    ("enable_wake",        "u8",    lambda v: v in (0, 1),       "0/1 (wake host on PS press)"),
    ("_reserved_d",        "res1",  None,                        ""),  # firmware trigger_reduce
    ("lock_volume",        "u8",    lambda v: v in (0, 1),       "0/1 (ignore the volume change from SetStateData(game or software))"),
    ("audio_follow_jack",  "u8",    lambda v: v in (0, 1),       "0/1 (default 1: show the USB audio device only while a headset is in the jack)"),
]
FIELD_NAMES = [f[0] for f in FIELDS]
# The DS4 settings the tool actually exposes (reserved padding excluded).
VISIBLE_FIELDS = [f for f in FIELDS if f[1] not in RESERVED_KINDS]
VISIBLE_NAMES = {f[0] for f in VISIBLE_FIELDS}

# Little-endian, no padding -- matches __attribute__((packed)) Config_body.
STRUCT_FMT = "<" + "".join(KIND_TO_CODE[f[1]] for f in FIELDS)
BODY_SIZE = struct.calcsize(STRUCT_FMT)
RESERVED_BYTES = sum(struct.calcsize(KIND_TO_CODE[f[1]])
                     for f in FIELDS if f[1] in RESERVED_KINDS)


def is_gamepad_hid(devinfo):
    return (devinfo.get("usage_page") == HID_USAGE_PAGE_GENERIC_DESKTOP and
            devinfo.get("usage") == HID_USAGE_GAMEPAD)


def fmt_hex(value):
    if value is None:
        return "?"
    return f"0x{int(value):04X}"


def describe_hid(devinfo):
    return (
        f"pid={fmt_hex(devinfo.get('product_id'))}, "
        f"interface={devinfo.get('interface_number', '?')}, "
        f"usage_page={fmt_hex(devinfo.get('usage_page'))}, "
        f"usage={fmt_hex(devinfo.get('usage'))}, "
        f"product={devinfo.get('product_string') or '?'}"
    )


def open_device():
    hid = _load_hid()
    cand = [d for d in hid.enumerate(VID) if d["product_id"] in PIDS]
    if not cand:
        sys.exit("No DS4Dongle / DualShock 4 found (VID 054C, PID 09CC). "
                 "Close Steam/DS4Windows if they're holding the device.")
    gamepads = [d for d in cand if is_gamepad_hid(d)]
    if not gamepads:
        # Linux hidraw often reports usage_page/usage as 0. Fall back to the
        # bridge's known gamepad interface number.
        gamepads = [d for d in cand if d.get("interface_number") == 3]
    if not gamepads:
        detail = "\n".join(f"  {describe_hid(d)}" for d in cand)
        sys.exit("Found DualSense / ds5dongle HID device(s), but none were the Game Pad interface "
                 "(usage_page=0x0001, usage=0x0005). Wake adds a keyboard HID; "
                 "this tool only opens the gamepad.\n" + detail)
    dev = hid.device()
    dev.open_path(gamepads[0]["path"])
    return dev


def _feature_read_help(report_id):
    # The config report IDs (0xF6-0xF9) are handled by the firmware but are not
    # declared in the DS4 HID report descriptor. Windows' HID class driver
    # rejects GET/SET_FEATURE for undeclared report IDs, which surfaces here as a
    # bare "read error". Give the user the real reason instead.
    msg = (f"Failed reading config report 0x{report_id:02X}.")
    if platform.system() == "Windows":
        msg += ("\n\nThis is expected on Windows: report IDs 0x{:02X}-0x{:02X} are not "
                "declared in the DS4 HID\nreport descriptor (kept byte-identical to a real "
                "DS4 v2), and Windows blocks\nGET/SET_FEATURE for any undeclared report ID. "
                "The DS4Dongle config tool only\nworks on Linux (hidraw passes the raw request "
                "through) unless the firmware is\nchanged to declare these reports as HID "
                "feature reports.").format(REPORT_SET, REPORT_GET_VERSION)
    return msg


def read_config(dev):
    # Windows hidapi expects the buffer to match the HID feature report length.
    # The config body is shorter than the descriptor report count, so read the
    # full report and unpack only Config_body.
    try:
        data = dev.get_feature_report(REPORT_GET_CONFIG, FEATURE_REPORT_LEN)
    except OSError as exc:
        sys.exit(f"{_feature_read_help(REPORT_GET_CONFIG)}\n\n(hidapi: {exc})")
    if not data:
        sys.exit("Empty response reading config (report 0xF7). Is the firmware current?")
    body = bytes(data[1:1 + BODY_SIZE]) if data[0] == REPORT_GET_CONFIG else bytes(data[:BODY_SIZE])
    if len(body) < BODY_SIZE:
        sys.exit(f"Short config read: got {len(body)} bytes, expected {BODY_SIZE}.")
    values = struct.unpack(STRUCT_FMT, body)
    return dict(zip(FIELD_NAMES, values))

def read_version(dev):
    try:
        data = dev.get_feature_report(REPORT_GET_VERSION, FEATURE_REPORT_LEN)
    except OSError:
        return ""
    raw = bytes(data[1:]) if data and data[0] == REPORT_GET_VERSION else bytes(data or b"")
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace").strip()

def write_config(dev, cfg, save):
    body = struct.pack(STRUCT_FMT, *[cfg[name] for name in FIELD_NAMES])
    # [report id][funcid 0x01][body...] padded to SET_DATA_LEN data bytes.
    data = bytes([FUNC_UPDATE]) + body
    data = data[:SET_DATA_LEN].ljust(SET_DATA_LEN, b"\x00")
    dev.send_feature_report(bytes([REPORT_SET]) + data)
    if save:
        save_data = bytes([FUNC_SAVE]).ljust(SET_DATA_LEN, b"\x00")
        dev.send_feature_report(bytes([REPORT_SET]) + save_data)


def fmt_value(name, value):
    return str(value)


def print_config(cfg):
    width = max(len(f[0]) for f in VISIBLE_FIELDS)
    for name, _kind, _ok, helptext in VISIBLE_FIELDS:
        print(f"  {name:<{width}} = {fmt_value(name, cfg[name]):<8}  # {helptext}")


def parse_assignment(token):
    if "=" not in token:
        sys.exit(f"Bad assignment '{token}', expected name=value.")
    name, raw = token.split("=", 1)
    name = name.strip()
    if name not in VISIBLE_NAMES:
        sys.exit(f"Unknown field '{name}'. Run 'config_tool.py fields' to list them.")
    if name == "config_version":
        sys.exit("config_version is managed by the firmware and cannot be set.")
    kind = dict((f[0], f[1]) for f in FIELDS)[name]
    validator = dict((f[0], f[2]) for f in FIELDS)[name]
    try:
        value = float(raw) if kind == "float" else int(raw, 0)
    except ValueError:
        sys.exit(f"Bad value '{raw}' for {name}.")
    if not validator(value):
        helptext = dict((f[0], f[3]) for f in FIELDS)[name]
        sys.exit(f"Value {raw} out of range for {name} (expected {helptext}).")
    return name, value


def cmd_fields(_args):
    width = max(len(f[0]) for f in VISIBLE_FIELDS)
    print(f"Config_body ({BODY_SIZE} bytes, schema version {CONFIG_VERSION}):")
    for name, kind, _ok, helptext in VISIBLE_FIELDS:
        ro = " (read-only)" if name == "config_version" else ""
        print(f"  {name:<{width}} {kind:<6} {helptext}{ro}")
    if RESERVED_BYTES:
        print(f"\n({RESERVED_BYTES} bytes are reserved for firmware settings the DS4 "
              f"does not use;\n they are kept in the {BODY_SIZE}-byte layout but not "
              "exposed.)")


def cmd_get(_args):
    dev = open_device()
    try:
        version = read_version(dev)
        cfg = read_config(dev)
    finally:
        dev.close()
    if version:
        print(f"Firmware: {version}")
    print("Config:")
    print_config(cfg)


def cmd_set(args):
    updates = dict(parse_assignment(t) for t in args.assignments)
    if not updates:
        sys.exit("Nothing to set. Pass one or more name=value pairs.")
    dev = open_device()
    try:
        cfg = read_config(dev)
        cfg.update(updates)
        write_config(dev, cfg, save=not args.no_save)
        new_cfg = read_config(dev)
    finally:
        dev.close()
    print("Updated:" + ("" if args.no_save else " (saved to flash)"))
    for name in updates:
        print(f"  {name} -> {fmt_value(name, new_cfg[name])}")
    # Firmware clamps invalid values; surface any that were adjusted.
    for name, want in updates.items():
        got = new_cfg[name]
        adjusted = abs(got - want) > 1e-6 if isinstance(want, float) else got != want
        if adjusted:
            print(f"  note: {name} was clamped by firmware to {fmt_value(name, got)}")


def main():
    parser = argparse.ArgumentParser(description="Read and modify ds5dongle config over USB HID.")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("get", help="read and print the current config").set_defaults(func=cmd_get)
    sub.add_parser("fields", help="list configurable fields and ranges").set_defaults(func=cmd_fields)

    p_set = sub.add_parser("set", help="set one or more fields (name=value ...)")
    p_set.add_argument("assignments", nargs="+", metavar="name=value")
    p_set.add_argument("--no-save", action="store_true",
                       help="update RAM only; do not persist to flash")
    p_set.set_defaults(func=cmd_set)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
