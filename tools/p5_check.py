#!/usr/bin/env python3
"""Bench check for MODE_PS5_AUTH without a PS5.

Plug either the bare P5General auth adapter or an OpenPuck in mode 13 (with the
adapter behind its MAX3421E) into a computer and run:

    uv run --no-project --with hid tools/p5_check.py

(macOS/Homebrew: prefix with DYLD_LIBRARY_PATH=/opt/homebrew/lib if hidapi
isn't found.)

Both enumerate as 2B81:0101. The bare adapter says "LiteEMF"; the puck says
"Activtor". For the adapter we sign a report ourselves; for the puck we just
read its stream, which only arrives if the adapter signed it (bytes 12..15 =
sequence, 56..63 = hash).
"""
import sys
import time

import hid

VID, PID = 0x2B81, 0x0101
DEF_03 = bytes.fromhex("2128 03c3 002c 5601 00d0 0700 8004 0000 800d 0d84")


def main():
    devs = hid.enumerate(VID, PID)
    if not devs:
        sys.exit("no 2B81:0101 device found")
    d = hid.Device(path=devs[0]["path"])
    mfr = d.manufacturer or ""
    print(f"found {mfr!r} / {d.product!r}")

    f3 = d.get_feature_report(0x03, 48)
    ok = f3[1 : 1 + len(DEF_03)] == DEF_03
    print(f"feature 0x03 definition: {'ok' if ok else 'MISMATCH ' + f3.hex()}")

    if mfr == "LiteEMF":
        plain = bytearray(64)
        plain[0] = 0x01
        plain[1:5] = b"\x80\x80\x80\x80"
        plain[8] = 0x0F
        plain[30] = 0x1A
        d.write(bytes(plain))
        r = d.read(64, timeout=500)
        signed = len(r) == 64 and any(r[56:64])
        print(f"adapter signing: {'ok' if signed else 'FAILED'} {r.hex()}")
        return

    hashes = set()
    t0 = time.time()
    n = 0
    while time.time() - t0 < 2.0:
        r = d.read(64, timeout=200)
        if len(r) == 64 and r[0] == 0x01:
            n += 1
            hashes.add(bytes(r[56:64]))
    rate = n / 2.0
    signed = n > 0 and len(hashes) > 1 and b"\0" * 8 not in hashes
    print(f"signed input stream: {'ok' if signed else 'FAILED'} "
          f"({rate:.0f} reports/s, {len(hashes)} distinct hashes)")
    if not n:
        print("no reports: is the adapter enumerating behind the MAX3421E? "
              "(puck LED solid = adapter seen, slow blink = not seen)")


if __name__ == "__main__":
    main()
