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
      funcid 0x08 + field  -> (opinionated) set a Wake-on-LAN field
      funcid 0x09          -> (opinionated) Wake-on-LAN test burst
  GET feature report 0xFA -> (opinionated) Wake-on-LAN config + status

Config_body is a packed struct; this tool derives the binary layout from FIELDS.

FIRMWARE VARIANTS:
  vanilla     (ds4-bridge.uf2) USB-identical to a real DS4 v2. The config report
              IDs 0xF6-0xF9 are handled but deliberately NOT declared in the HID
              report descriptor, and Windows' HID class driver rejects
              GET/SET_FEATURE for any undeclared report ID -- so on Windows
              every command fails with "read error". Works on Linux (hidraw
              passes the raw request through regardless of the descriptor).
  opinionated (ds4-bridge-opinionated.uf2) declares 0xF6-0xFA, so this tool
              works on Windows too, and adds Wi-Fi Wake-on-LAN (wol_* fields).

Requires: pip install hidapi

Examples:
  python config_tool.py get
  python config_tool.py set speaker_volume=90 enable_wake=1
  python config_tool.py set inactive_time=10 --no-save
  python config_tool.py fields

Wake-on-LAN (opinionated firmware):
  python config_tool.py set wol_ssid=MyWifi wol_password=- wol_mac=aa:bb:cc:dd:ee:ff wol_enabled=1
  python config_tool.py wol-test
  (wol_password=- prompts for the password instead of leaving it in shell history)
"""
import platform
import argparse
import getpass
import struct
import sys
import time


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
REPORT_GET_WOL = 0xFA    # GET_REPORT: (opinionated) Wake-on-LAN config + status

FUNC_UPDATE = 0x01       # update config in RAM
FUNC_SAVE = 0x02         # persist to flash
FUNC_RECONNECT = 0x03    # reconnect tinyusb device
FUNC_WOL_SET = 0x08      # (opinionated) set a Wake-on-LAN field
FUNC_WOL_TEST = 0x09     # (opinionated) join Wi-Fi + short magic-packet burst

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

# Wake-on-LAN settings (opinionated firmware, src/wol.cpp). Not part of
# Config_body: set field-wise via funcid 0x08, read via 0xFA. The password is
# write-only -- the firmware only reports its length.
WOL_FIELD_ENABLED, WOL_FIELD_MAC, WOL_FIELD_SSID, WOL_FIELD_PASSWORD = range(4)
WOL_SSID_MAX = 32
WOL_PASSWORD_MAX = 63
WOL_CHUNK = SET_DATA_LEN - 4  # after [funcid][field][offset][len]
WOL_FIELDS = [
    ("wol_enabled",  "0/1 (Wi-Fi Wake-on-LAN when the controller connects and the PC is not up)"),
    ("wol_ssid",     f"Wi-Fi network name, 2.4 GHz (max {WOL_SSID_MAX} bytes)"),
    ("wol_password", f"WPA2 password (8..{WOL_PASSWORD_MAX}, empty = open network; write-only, '-' prompts)"),
    ("wol_mac",      "MAC address of the PC's network card, aa:bb:cc:dd:ee:ff"),
]
WOL_NAMES = {f[0] for f in WOL_FIELDS}
WOL_STATES = ["idle (Wi-Fi off)", "joining Wi-Fi",
              "sending magic packets", "retrying Wi-Fi join"]
WOL_RESULTS = ["-", "stopped: PC is up (USB data connection)", "gave up: timeout",
               "test finished", "not configured / disabled",
               "Wi-Fi chip stopped responding -- dongle reboots to recover"]
WOL_LINK = {0: "down", 1: "joined", -1: "join failed", -2: "SSID not found", -3: "auth failed (wrong password, or WPA handshake timed out -- retried)"}


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
    # The vanilla firmware handles the config report IDs (0xF6-0xF9) but does not
    # declare them in the DS4 HID report descriptor. Windows' HID class driver
    # rejects GET/SET_FEATURE for undeclared report IDs, which surfaces here as a
    # bare "read error". Give the user the real reason instead.
    msg = (f"Failed reading config report 0x{report_id:02X}.")
    if platform.system() == "Windows":
        msg += ("\n\nThis is expected on Windows with the vanilla firmware: report IDs "
                "0x{:02X}-0x{:02X} are not\ndeclared in its HID report descriptor (kept "
                "byte-identical to a real DS4 v2), and\nWindows blocks GET/SET_FEATURE for "
                "any undeclared report ID. Flash the opinionated\nfirmware "
                "(ds4-bridge-opinionated.uf2), which declares them, or configure from "
                "Linux.").format(REPORT_SET, REPORT_GET_VERSION)
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


def read_wol(dev):
    """Wake-on-LAN config + status (0xFA, layout in src/wol.cpp), or None on
    firmware without it (vanilla stalls the request)."""
    try:
        data = dev.get_feature_report(REPORT_GET_WOL, FEATURE_REPORT_LEN)
    except OSError:
        return None
    raw = bytes(data[1:]) if data and data[0] == REPORT_GET_WOL else bytes(data or b"")
    if len(raw) < 48 or raw[0] != 1:
        return None
    return {
        "wol_enabled": raw[1],
        "wol_mac": ":".join(f"{b:02x}" for b in raw[2:8]),
        "wol_ssid": raw[9:9 + min(raw[8], WOL_SSID_MAX)].decode("utf-8", "replace"),
        "wol_password": f"<set, {raw[41]} chars>" if raw[41] else "<none: open network>",
        "state": raw[42],
        "link": raw[43] - 256 if raw[43] > 127 else raw[43],
        "result": raw[44],
        "packets": raw[45] | raw[46] << 8,
        "joins": raw[47],
    }


def send_set(dev, funcid, payload=b""):
    # [report id][funcid][payload...] padded to SET_DATA_LEN data bytes.
    data = (bytes([funcid]) + payload)[:SET_DATA_LEN].ljust(SET_DATA_LEN, b"\x00")
    dev.send_feature_report(bytes([REPORT_SET]) + data)


def write_config(dev, cfg, save):
    body = struct.pack(STRUCT_FMT, *[cfg[name] for name in FIELD_NAMES])
    send_set(dev, FUNC_UPDATE, body)
    if save:
        send_set(dev, FUNC_SAVE)


def write_wol_field(dev, field, value):
    # [field][offset][len][data...]; strings go in chunks, offset 0 starts a new
    # value and a zero-length chunk at offset 0 clears it.
    offset = 0
    while True:
        chunk = value[offset:offset + WOL_CHUNK]
        send_set(dev, FUNC_WOL_SET, bytes([field, offset, len(chunk)]) + chunk)
        offset += len(chunk)
        if offset >= len(value):
            break


def parse_mac(text):
    if ":" in text or "-" in text:
        parts = text.replace("-", ":").split(":")
    else:
        parts = [text[i:i + 2] for i in range(0, len(text), 2)]
    try:
        mac = bytes(int(p, 16) for p in parts if len(p) == 2)
    except ValueError:
        mac = b""
    if len(mac) != 6 or len(parts) != 6:
        sys.exit(f"Bad MAC address '{text}', expected aa:bb:cc:dd:ee:ff.")
    return mac


def parse_wol_assignment(name, raw):
    """-> (field id, value bytes)"""
    if name == "wol_enabled":
        if raw not in ("0", "1"):
            sys.exit("wol_enabled must be 0 or 1.")
        return WOL_FIELD_ENABLED, bytes([int(raw)])
    if name == "wol_mac":
        return WOL_FIELD_MAC, parse_mac(raw)
    if name == "wol_ssid":
        value = raw.encode("utf-8")
        if not 1 <= len(value) <= WOL_SSID_MAX:
            sys.exit(f"wol_ssid must be 1..{WOL_SSID_MAX} bytes.")
        return WOL_FIELD_SSID, value
    if name == "wol_password":
        if raw == "-":
            raw = getpass.getpass("Wi-Fi password (empty = open network): ")
        value = raw.encode("utf-8")
        if value and not 8 <= len(value) <= WOL_PASSWORD_MAX:
            sys.exit(f"wol_password must be 8..{WOL_PASSWORD_MAX} bytes (WPA2), "
                     "or empty for an open network.")
        return WOL_FIELD_PASSWORD, value
    raise AssertionError(name)


def fmt_value(name, value):
    return str(value)


def print_config(cfg):
    width = max(len(f[0]) for f in VISIBLE_FIELDS)
    for name, _kind, _ok, helptext in VISIBLE_FIELDS:
        print(f"  {name:<{width}} = {fmt_value(name, cfg[name]):<8}  # {helptext}")


def print_wol(wol):
    width = max(len(f[0]) for f in VISIBLE_FIELDS)
    for name, helptext in WOL_FIELDS:
        print(f"  {name:<{width}} = {wol[name]!s:<8}  # {helptext}")


def print_wol_status(wol):
    def lookup(table, key):
        return table[key] if key < len(table) else key
    print(f"  status: {lookup(WOL_STATES, wol['state'])}; "
          f"Wi-Fi {WOL_LINK.get(wol['link'], wol['link'])}; "
          f"{wol['packets']} magic packets, {wol['joins']} join attempts; "
          f"last result: {lookup(WOL_RESULTS, wol['result'])}")


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
    print("\nWake-on-LAN (opinionated firmware only, stored separately from Config_body):")
    for name, helptext in WOL_FIELDS:
        print(f"  {name:<{width}} {'str':<6} {helptext}")
    if RESERVED_BYTES:
        print(f"\n({RESERVED_BYTES} bytes are reserved for firmware settings the DS4 "
              f"does not use;\n they are kept in the {BODY_SIZE}-byte layout but not "
              "exposed.)")


def cmd_get(_args):
    dev = open_device()
    try:
        version = read_version(dev)
        cfg = read_config(dev)
        wol = read_wol(dev)
    finally:
        dev.close()
    if version:
        print(f"Firmware: {version}")
    print("Config:")
    print_config(cfg)
    if wol is not None:
        print("Wake-on-LAN:")
        print_wol(wol)
        print_wol_status(wol)


def cmd_set(args):
    updates = {}
    wol_updates = []  # (name, field id, value bytes)
    for token in args.assignments:
        name, _, raw = token.partition("=")
        if name.strip() in WOL_NAMES and _:
            wol_updates.append((name.strip(),) + parse_wol_assignment(name.strip(), raw))
        else:
            key, value = parse_assignment(token)
            updates[key] = value
    if not updates and not wol_updates:
        sys.exit("Nothing to set. Pass one or more name=value pairs.")
    dev = open_device()
    try:
        cfg = read_config(dev)
        if wol_updates and read_wol(dev) is None:
            sys.exit("This firmware has no Wake-on-LAN -- the wol_* fields need the "
                     "opinionated build (ds4-bridge-opinionated.uf2).")
        # Wake-on-LAN fields first, so the save below persists them together
        # with Config_body (one flash page).
        for _name, field, value in wol_updates:
            write_wol_field(dev, field, value)
        cfg.update(updates)
        write_config(dev, cfg, save=not args.no_save)
        new_cfg = read_config(dev)
        new_wol = read_wol(dev) if wol_updates else None
    finally:
        dev.close()
    print("Updated:" + ("" if args.no_save else " (saved to flash)"))
    for name in updates:
        print(f"  {name} -> {fmt_value(name, new_cfg[name])}")
    for name, _field, _value in wol_updates:
        print(f"  {name} -> {new_wol[name]}")
    if new_wol and new_wol["wol_enabled"] and (
            not new_wol["wol_ssid"] or new_wol["wol_mac"] == "00:00:00:00:00:00"):
        print("  note: Wake-on-LAN is enabled but has no wol_ssid / wol_mac yet -- it stays "
              "inactive until both are set.")
    # Firmware clamps invalid values; surface any that were adjusted.
    for name, want in updates.items():
        got = new_cfg[name]
        adjusted = abs(got - want) > 1e-6 if isinstance(want, float) else got != want
        if adjusted:
            print(f"  note: {name} was clamped by firmware to {fmt_value(name, got)}")


def cmd_wol_test(_args):
    dev = open_device()
    try:
        wol = read_wol(dev)
        if wol is None:
            sys.exit("This firmware has no Wake-on-LAN (flash ds4-bridge-opinionated.uf2).")
        if not wol["wol_ssid"] or wol["wol_mac"] == "00:00:00:00:00:00":
            sys.exit("Wake-on-LAN is not configured: set wol_ssid (and wol_password, wol_mac) "
                     "first, e.g.\n  python config_tool.py set \"wol_ssid=MyWifi\" wol_password=-")
        print(f"Wake-on-LAN test: joining '{wol['wol_ssid']}' and sending magic packets "
              f"to {wol['wol_mac']} ...")
        send_set(dev, FUNC_WOL_TEST)
        time.sleep(0.5)
        deadline = time.time() + 35
        last = None
        while time.time() < deadline:
            fresh = read_wol(dev)
            if fresh is None:
                sys.exit("The dongle stopped answering the status read (0xFA) mid-test. "
                         f"Last status: {wol['packets']} magic packets sent.")
            wol = fresh
            snapshot = (wol["state"], wol["link"], wol["packets"], wol["result"])
            if snapshot != last:
                print_wol_status(wol)
                last = snapshot
            if wol["state"] == 0 and wol["result"] != 0:
                break
            time.sleep(0.5)
    finally:
        dev.close()
    if wol["packets"]:
        print("OK: joined Wi-Fi and sent magic packets. A running PC ignores them; to see them "
              "arrive,\ncapture on the PC (Wireshark display filter: wol).")
    else:
        print("No magic packets were sent -- check wol_ssid / wol_password; the network must be "
              "2.4 GHz WPA2 (or open).")


def main():
    parser = argparse.ArgumentParser(description="Read and modify DS4Dongle config over USB HID.")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("get", help="read and print the current config").set_defaults(func=cmd_get)
    sub.add_parser("fields", help="list configurable fields and ranges").set_defaults(func=cmd_fields)

    p_set = sub.add_parser("set", help="set one or more fields (name=value ...)")
    p_set.add_argument("assignments", nargs="+", metavar="name=value")
    p_set.add_argument("--no-save", action="store_true",
                       help="update RAM only; do not persist to flash")
    p_set.set_defaults(func=cmd_set)

    sub.add_parser("wol-test", help="(opinionated) join Wi-Fi now and send a few magic packets"
                   ).set_defaults(func=cmd_wol_test)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
