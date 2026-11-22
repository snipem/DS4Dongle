#!/usr/bin/env python3
"""
Flash a .uf2 onto the DS4Dongle end-to-end and verify it took.

The full BOOTSEL track:
  1. If the RP2350/RP2040 BOOTSEL drive is already mounted, use it. Otherwise
     send the reboot-to-BOOTSEL command (feature report 0xF6, funcid 0x04) to
     the running firmware. If that can't be delivered (vanilla firmware on
     Windows drops undeclared report ids) ask for the physical BOOTSEL button
     and keep waiting.
  2. Wait for the BOOTSEL mass-storage drive (identified by INFO_UF2.TXT).
  3. Copy the .uf2 onto it.
  4. Wait for the drive to disappear -- the bootloader only reboots once it
     has accepted every block of the image.
  5. Wait for the dongle to re-enumerate as a DS4 (054C:09CC) and read the
     firmware version string (feature report 0xF8); it must match
     --expect-version. The firmware only brings up USB once a DualShock 4 is
     connected over Bluetooth, so this step prompts you to switch one on and
     waits --enum-timeout seconds (default 120).

On Windows only the opinionated firmware answers 0xF8 (vanilla doesn't declare
it), so for vanilla builds step 5 can only confirm re-enumeration and prints a
warning instead of a version match.

Requires: pip install hidapi
Usage:    python flash.py <file.uf2> [--expect-version VERSION] [--timeout 30]
"""
import argparse
import glob
import os
import platform
import shutil
import string
import subprocess
import sys
import time

from config_tool import (VID, PIDS, REPORT_SET, SET_DATA_LEN, is_gamepad_hid,
                         read_version, _load_hid)

FUNCID_REBOOT_BOOTSEL = 0x04
BOOTSEL_LABELS = ("RP2350", "RPI-RP2")


def log(msg):
    print(f"flash: {msg}", flush=True)


# --- BOOTSEL drive discovery ------------------------------------------------

def _is_uf2_drive(root):
    return os.path.isfile(os.path.join(root, "INFO_UF2.TXT"))


def _linux_mount_bootsel():
    """Mount an unmounted BOOTSEL block device via udisksctl (best effort)."""
    for label in BOOTSEL_LABELS:
        try:
            dev = subprocess.run(["blkid", "-t", f"LABEL={label}", "-o", "device"],
                                 capture_output=True, text=True).stdout.split()
        except FileNotFoundError:
            return
        if dev:
            subprocess.run(["udisksctl", "mount", "-b", dev[0], "--no-user-interaction"],
                           capture_output=True)
            return


def find_bootsel_drive():
    system = platform.system()
    if system == "Windows":
        candidates = [f"{c}:\\" for c in string.ascii_uppercase if c not in "AB"]
    elif system == "Darwin":
        candidates = glob.glob("/Volumes/*")
    else:
        candidates = (glob.glob("/media/*/*") + glob.glob("/run/media/*/*") +
                      glob.glob("/media/*") + glob.glob("/mnt/*"))
    for root in candidates:
        try:
            if _is_uf2_drive(root):
                return root
        except OSError:
            continue
    if system == "Linux":
        _linux_mount_bootsel()
    return None


def bootsel_info(root):
    try:
        with open(os.path.join(root, "INFO_UF2.TXT"), encoding="ascii", errors="replace") as f:
            return " / ".join(l.strip() for l in f if l.strip())
    except OSError:
        return "?"


# --- HID helpers --------------------------------------------------------------

def find_gamepad(hid):
    cand = [d for d in hid.enumerate(VID) if d["product_id"] in PIDS]
    pads = [d for d in cand if is_gamepad_hid(d)] or \
           [d for d in cand if d.get("interface_number") == 3]
    return pads[0] if pads else None


def send_reboot(hid, info):
    """Returns True if the command was (very likely) delivered."""
    dev = hid.device()
    try:
        dev.open_path(info["path"])
    except OSError as e:
        log(f"could not open HID device ({e}) -- is Steam/DS4Windows holding it?")
        return False
    payload = bytes([REPORT_SET, FUNCID_REBOOT_BOOTSEL]).ljust(SET_DATA_LEN + 1, b"\x00")
    try:
        result = dev.send_feature_report(payload)
    except (OSError, ValueError):
        return True  # device dropped off the bus mid-request: it's rebooting
    finally:
        try:
            dev.close()
        except Exception:
            pass
    return result is None or result >= 0


def read_version_from(hid, info, attempts=10):
    for _ in range(attempts):
        dev = hid.device()
        try:
            dev.open_path(info["path"])
            v = read_version(dev)
            if v:
                return v
        except OSError:
            pass
        finally:
            try:
                dev.close()
            except Exception:
                pass
        time.sleep(0.5)
    return ""


def wait_for(predicate, timeout, interval=0.25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        r = predicate()
        if r:
            return r
        time.sleep(interval)
    return None


# --- main -------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("uf2")
    ap.add_argument("--expect-version", default="",
                    help="firmware version string the new image must report")
    ap.add_argument("--timeout", type=float, default=30,
                    help="seconds to wait for the BOOTSEL drive to appear/disappear")
    ap.add_argument("--enum-timeout", type=float, default=120,
                    help="seconds to wait for the dongle to come back on USB after "
                         "flashing (needs a DualShock 4 connected)")
    args = ap.parse_args()

    if not os.path.isfile(args.uf2):
        sys.exit(f"flash: no such file: {args.uf2}")
    hid = _load_hid()

    # 1. Get into BOOTSEL.
    drive = find_bootsel_drive()
    if drive:
        log(f"BOOTSEL drive already present at {drive}")
    else:
        pad = find_gamepad(hid)
        if pad is None:
            log("no DS4Dongle on USB (it only enumerates while a DualShock 4 is "
                "connected) -- switch the controller on, or hold BOOTSEL while "
                "plugging the dongle in")
        else:
            if pad.get("usage") == 0x05:
                old = read_version_from(hid, pad, attempts=1)
                if old:
                    log(f"currently running: {old}")
            log("sending reboot-to-BOOTSEL (0xF6/0x04)")
            if not send_reboot(hid, pad):
                log("firmware did not accept the reboot command (vanilla firmware on "
                    "Windows?) -- press/hold BOOTSEL and replug the dongle")

        # 2. Wait for the mass-storage drive.
        log(f"waiting up to {args.timeout:.0f}s for the BOOTSEL drive...")
        drive = wait_for(find_bootsel_drive, args.timeout, 0.5)
        if not drive:
            sys.exit(f"flash: FAILED -- no BOOTSEL drive appeared within {args.timeout:.0f}s")
        log(f"BOOTSEL drive at {drive}")
    log(f"bootloader: {bootsel_info(drive)}")

    # 3. Copy the image.
    size = os.path.getsize(args.uf2)
    log(f"copying {os.path.basename(args.uf2)} ({size // 1024} KiB) -> {drive}")
    dest = os.path.join(drive, os.path.basename(args.uf2))
    try:
        with open(args.uf2, "rb") as src, open(dest, "wb") as dst:
            shutil.copyfileobj(src, dst, 1 << 16)
            dst.flush()
            os.fsync(dst.fileno())
    except OSError as e:
        # The bootloader may reboot (and the drive vanish) right after the last
        # block lands, which can surface as an error on close. Step 4 decides.
        log(f"copy reported {e.__class__.__name__}: {e} (checking whether the device rebooted)")

    # 4. Drive must disappear: the bootloader reboots only after a complete image.
    log("waiting for the bootloader to accept the image and reboot...")
    if not wait_for(lambda: not _drive_still_there(drive), args.timeout):
        sys.exit("flash: FAILED -- BOOTSEL drive is still present; the image was not accepted "
                 "(wrong chip family or truncated .uf2?)")
    log("bootloader accepted the image")

    # 5. Re-enumeration + version check.
    log(f">>> switch on / reconnect your DualShock 4 now -- the dongle only shows up "
        f"on USB once a controller is connected (waiting up to {args.enum_timeout:.0f}s)")
    pad = wait_for(lambda: find_gamepad(hid), args.enum_timeout, 0.5)
    if pad is None:
        sys.exit(f"flash: FAILED -- the dongle did not come back as a DS4 (054C:09CC) "
                 f"within {args.enum_timeout:.0f}s. The image was accepted by the "
                 f"bootloader; if no controller connected, the firmware may be fine -- "
                 f"re-check with: python tools/config_tool.py get")
    log(f"dongle is back ({pad.get('product_string') or 'DS4'})")

    version = read_version_from(hid, pad)
    if not version:
        msg = "could not read the firmware version (report 0xF8)"
        if platform.system() == "Windows":
            msg += " -- expected for the vanilla firmware on Windows"
        log(f"WARNING: {msg}; flash verified only by re-enumeration")
        return
    log(f"firmware reports version: {version}")
    if args.expect_version and version != args.expect_version:
        sys.exit(f"flash: FAILED -- expected version '{args.expect_version}', "
                 f"device reports '{version}'")
    log("SUCCESS" + (" -- version matches" if args.expect_version else ""))


def _drive_still_there(root):
    try:
        return _is_uf2_drive(root)
    except OSError:
        return False


if __name__ == "__main__":
    main()
