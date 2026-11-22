#!/usr/bin/env python3
import glob
import os
import plistlib
import subprocess
import sys

PATTERNS = ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.SLAB*", "/dev/ttyACM*", "/dev/ttyUSB*")


def serial_ports():
    return sorted(p for pat in PATTERNS for p in glob.glob(pat))


def callouts(node):
    for child in node.get("IORegistryEntryChildren", []):
        if child.get("IOObjectClass") == "IOUSBHostDevice":
            continue
        if "IOCalloutDevice" in child:
            yield child["IOCalloutDevice"]
        yield from callouts(child)


def describe_mac():
    try:
        out = subprocess.run(["ioreg", "-a", "-r", "-c", "IOUSBHostDevice", "-l"], capture_output=True, check=True).stdout
        devices = plistlib.loads(out) if out.strip() else []
    except (OSError, subprocess.CalledProcessError, plistlib.InvalidFileException):
        return {}
    info = {}
    for d in devices:
        what = " ".join(x for x in (d.get("USB Vendor Name"), d.get("USB Product Name")) if x) or "unknown USB device"
        serial = d.get("USB Serial Number", "")
        for port in callouts(d):
            info[port] = (what, serial)
    return info


def describe_linux():
    info = {}
    for link in glob.glob("/dev/serial/by-id/*"):
        info[os.path.realpath(link)] = (os.path.basename(link), "")
    return info


def main():
    ports = serial_ports()
    if not ports:
        print("no USB serial devices. A board that shows none: hold BOOT (B) while plugging the USB in; else try another (data) cable or USB port.")
        return 1
    info = describe_mac() if sys.platform == "darwin" else describe_linux()
    for p in ports:
        what, serial = info.get(p, ("", ""))
        tag = "  <- ESP32-S3 USB (SuperMini or XIAO)" if "Espressif" in what or "JTAG" in what else ""
        print(f"{p}  {what}" + (f"  serial {serial}" if serial else "") + tag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
