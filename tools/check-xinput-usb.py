#!/usr/bin/env python3
"""Opt-in physical OpenPuck XSM3 transport check, not a console acceptance test."""

import argparse
import hashlib
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import sys
from threading import Event
import time


USB_TIMEOUT_MS = 1500
READY_TIMEOUT_S = 5.0
POLL_INTERVAL_S = 0.010
SECURITY = (
    "Xbox Security Method 3, Version 1.00, © 2005 Microsoft "
    "Corporation. All rights reserved."
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def positive_int(value):
    number = int(value, 10)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive decimal integer")
    return number


def load_reference():
    path = Path(__file__).resolve().parents[1] / "tests/xinput/reference.py"
    spec = importlib.util.spec_from_file_location("xinput_usb_reference", path)
    require(spec is not None and spec.loader is not None,
            "cannot load independent reference")
    reference = importlib.util.module_from_spec(spec)
    # Execute source directly so the manual checker never creates a pycache.
    exec(compile(path.read_bytes(), str(path), "exec"), reference.__dict__)
    return reference


class Probe:
    def __init__(self):
        self.stage = "setup"
        self.device = None
        self.waiter = Event()

    def progress(self, stage):
        self.stage = stage
        print(f"[{stage}]", flush=True)

    def descriptor(self, kind, index, length, language=0):
        return bytes(self.device.ctrl_transfer(
            0x80, 6, (kind << 8) | index, language, length,
            timeout=USB_TIMEOUT_MS))

    def string(self, index):
        require(index != 0, "missing required string index")
        raw = self.descriptor(3, index, 255, 0x0409)
        require(len(raw) >= 2 and raw[0] == len(raw) and raw[1] == 3
                and len(raw) % 2 == 0, "malformed or truncated string")
        return raw[2:].decode("utf-16-le")

    def gate(self):
        dev = self.device
        require((dev.bcdUSB, dev.bcdDevice, dev.bDeviceClass,
                 dev.bDeviceSubClass, dev.bDeviceProtocol,
                 dev.bMaxPacketSize0, dev.bNumConfigurations)
                == (0x0200, 0x0114, 0xFF, 0xFF, 0xFF, 64, 1),
                "device descriptor is not the expected OpenPuck personality")
        config = self.descriptor(2, 0, 153)
        require(len(config) == 153 and config[:9] == bytes(
            [9, 2, 153, 0, 4, 1, 0, 0xA0, 250]),
            "expected a 153-byte, four-interface OpenPuck configuration")
        interfaces = []
        offset = 9
        while offset < len(config):
            size = config[offset]
            require(size >= 2 and offset + size <= len(config),
                    "malformed configuration descriptor")
            if config[offset + 1] == 4:
                require(size == 9, "malformed interface descriptor")
                interfaces.append(tuple(config[offset + 2:offset + 9]))
            offset += size
        require(interfaces == [
            (0, 0, 2, 0xFF, 0x5D, 1, 0),
            (1, 0, 4, 0xFF, 0x5D, 3, 0),
            (2, 0, 1, 0xFF, 0x5D, 2, 0),
            (3, 0, 0, 0xFF, 0xFD, 0x13, 4),
        ], "interface layout is not the expected OpenPuck personality")
        require(self.string(dev.iManufacturer) == "©Microsoft Corporation"
                and self.string(dev.iProduct) == "Controller",
                "manufacturer/product strings do not match")
        require(self.string(4) == SECURITY, "full security string mismatch")
        serial = self.string(dev.iSerialNumber)
        # Firmware formats the hardware ID as %04lX%08lX; no fixed prefix.
        require(re.fullmatch(r"[0-9A-F]{12}", serial) is not None,
                "expected a 12-character uppercase hexadecimal serial")
        print("PASS descriptor/string gates (serial withheld)", flush=True)
        return serial.encode("ascii")

    def read(self, request, length, value=0, timeout=USB_TIMEOUT_MS):
        data = bytes(self.device.ctrl_transfer(
            0xC1, request, value, 0x0103, length, timeout=timeout))
        require(len(data) == length,
                f"IN 0x{request:02x}: expected {length} bytes, got {len(data)}")
        return data

    def write(self, request, data):
        count = self.device.ctrl_transfer(
            0x41, request, 3, 0x0103, data, timeout=USB_TIMEOUT_MS)
        require(count == len(data), f"OUT 0x{request:02x}: short transfer")

    def exchange(self, request, challenge, length):
        started = time.monotonic()
        self.write(request, challenge)
        deadline = started + READY_TIMEOUT_S
        polls = 0
        while True:
            remaining = deadline - time.monotonic()
            require(remaining > 0, "authentication readiness deadline expired")
            status = self.read(0x86, 2, timeout=min(
                USB_TIMEOUT_MS, max(1, int(remaining * 1000))))
            polls += 1
            require(time.monotonic() <= deadline,
                    "authentication readiness deadline expired")
            if status == b"\x02\x00":
                ready_ms = (time.monotonic() - started) * 1000
                response = self.read(0x83, length, 0x5C00 | (length - 6))
                print(f"ready in {ready_ms:.1f} ms ({polls} polls); "
                      f"response in {(time.monotonic() - started) * 1000:.1f} ms",
                      flush=True)
                return response
            require(status == b"\x01\x00",
                    f"unexpected authentication status {status.hex()}")
            self.waiter.wait(min(POLL_INTERVAL_S,
                                 max(0, deadline - time.monotonic())))

    def authenticate(self, reference, serial, skip_keepalive):
        r = reference
        self.progress("identification: IN 0x81 (invalidates active session)")
        identification = self.read(0x81, 29, 0x5B17)
        require(identification == r.packet(
            serial + bytes.fromhex("0080025e048e0203000101"), b"IK"),
            "identification framing, serial or identity mismatch")
        identity = bytearray(32)
        identity[:15] = identification[5:20]
        identity[16:21] = identification[20:25]
        identity[21] = identification[27]
        identity[22:24] = identification[25:27]

        # Synthetic public-key oracle inputs, never console secrets/captures.
        console = bytes(range(16, 24))
        nonce = bytes(range(16))
        digest = hashlib.sha1(console).digest()
        session1 = r.crypt(r.crypt(r.ROOT_23, digest[:16]), nonce)
        session2 = r.crypt(r.crypt(r.ROOT_24, digest[4:]), nonce[8:] + nonce[:8])
        encrypted = r.crypt(r.KEY_D, nonce + console)
        self.progress("init: OUT 0x82 → status → response")
        reply = self.exchange(
            0x82, r.packet(encrypted + r.mac(r.KEY_E, encrypted)[4:]), 46)
        recovered = r.crypt(session1, reply[5:37], False)
        require(recovered[16:] == nonce, "init recovered console nonce mismatch")
        require(reply == r.packet(reply[5:37] + r.acr(
            console, identity, r.mac(session2, reply[5:37]))),
            "init response differs from independent OpenSSL/Python oracle")
        print("PASS init", flush=True)

        self.progress("keepalive: zero-data OUT 0x84")
        if skip_keepalive:
            print("SKIPPED: --skip-keepalive diagnostic old-build test; "
                  "keepalive NOT validated", flush=True)
        else:
            started = time.monotonic()
            self.write(0x84, b"")
            print(f"PASS keepalive ({(time.monotonic() - started) * 1000:.1f} ms)",
                  flush=True)

        controller = recovered[:16]
        init_hash = hashlib.sha1(recovered).digest()
        salt = bytearray(controller[12:] + nonce[12:] + nonce[8:])
        for iteration in range(3):
            self.progress(f"verify {iteration + 1}/3: OUT 0x87 → status → response")
            fresh = bytes(range(0x60 + iteration * 8, 0x68 + iteration * 8))
            salt[8:] = fresh
            encrypted = r.crypt(controller, fresh)
            challenge = r.packet(encrypted + r.mac(init_hash, encrypted, salt))
            reply = self.exchange(0x87, challenge, 22)
            encrypted = r.crypt(session1, r.acr(console, identity, fresh))
            require(reply == r.packet(encrypted + r.mac(session2, encrypted, salt)),
                    "verify response differs from independent OpenSSL/Python oracle")
            print(f"PASS verify {iteration + 1}/3", flush=True)


def main():
    parser = argparse.ArgumentParser(
        description="Manual PC USB authentication check for an attached OpenPuck "
        "045e:028e, not a genuine controller. --run invalidates any active "
        "authentication session; unplug/replug afterward, even on failure.",
        epilog="Requires PyUSB, a libusb backend, USB permissions, and OpenSSL. "
        "Never detaches drivers, sets configuration, resets or flashes hardware. "
        "Descriptor/string gates reduce accidental targeting but cannot prove "
        "device provenance: connect only the intended OpenPuck.")
    parser.add_argument("--run", action="store_true",
                        help="explicitly permit device access and authentication")
    parser.add_argument("--bus", type=positive_int, help="USB bus (decimal)")
    parser.add_argument("--address", type=positive_int, help="USB address (decimal)")
    parser.add_argument("--skip-keepalive", action="store_true",
                        help="DIAGNOSTIC old-build test only: skip zero-data OUT "
                        "0x84; does not validate the keepalive fix")
    args = parser.parse_args()
    if not args.run:
        print("No USB access performed. Use --run explicitly; see --help.")
        return 1

    probe = Probe()
    try:
        try:
            import usb.core
            import usb.util
        except ImportError:
            raise RuntimeError("PyUSB unavailable; install the pyusb package "
                               "in your Python environment") from None
        require(shutil.which("openssl") is not None, "OpenSSL executable missing")
        reference = load_reference()
        probe.progress("select exactly one 045e:028e (optional bus/address filter)")
        devices = list(usb.core.find(
            find_all=True, idVendor=0x045E, idProduct=0x028E))
        devices = [dev for dev in devices
                   if (args.bus is None or dev.bus == args.bus)
                   and (args.address is None or dev.address == args.address)]
        require(len(devices) == 1,
                f"found {len(devices)} matching devices; require exactly one "
                "(use --bus/--address to disambiguate)")
        probe.device = devices[0]
        probe.progress("read-only descriptor/string gates")
        try:
            serial = probe.gate()
            print("WARNING: authentication invalidates the active session. "
                  "Unplug/replug afterward, including on failure.", flush=True)
            probe.authenticate(reference, serial, args.skip_keepalive)
        finally:
            usb.util.dispose_resources(probe.device)
        print("PASS PC USB synthetic authentication" +
              (" (DIAGNOSTIC: keepalive skipped)" if args.skip_keepalive else "") +
              "; console compatibility remains unverified. Unplug/replug now.")
        return 0
    except KeyboardInterrupt:
        print(f"ERROR [{probe.stage}]: interrupted; unplug/replug afterward.",
              file=sys.stderr)
        return 1
    except Exception as error:
        # CalledProcessError includes the OpenSSL command/key; don't log it.
        detail = ("OpenSSL reference operation failed"
                  if isinstance(error, subprocess.CalledProcessError)
                  else str(error))
        print(f"ERROR [{probe.stage}]: {detail}; unplug/replug if authentication "
              "started.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())