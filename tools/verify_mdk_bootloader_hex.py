#!/usr/bin/env python3
"""Validate the OpenPuck Makerdiary bootloader Intel HEX before DFU packaging.

The Adafruit nRF52840 bootloader HEX intentionally contains two UICR words:
  0x10001014  UICR_BOOTLOADER      -> 0x000F4000
  0x10001018  UICR_MBR_PARAM_PAGE -> 0x000FE000

Those are part of Adafruit's standard nRF52840 linker layout and are required
for the MBR/bootloader arrangement.  No other UICR or out-of-flash addresses
are accepted by this validator.
"""

from __future__ import annotations
import argparse
from pathlib import Path

BOOTLOADER_BASE = 0x000F4000
BOOTLOADER_DATA_END = 0x000FE000  # exclusive; MBR params/settings are NOLOAD
EXPECTED_UF2_MARKER = b"UF2 Bootloader 0.7.1-openpuck"

UICR_BOOTLOADER = 0x10001014
UICR_MBR_PARAM_PAGE = 0x10001018

EXPECTED_UICR_WORDS = {
    UICR_BOOTLOADER: BOOTLOADER_BASE,
    UICR_MBR_PARAM_PAGE: 0x000FE000,
}

ALLOWED_UICR_BYTES = {
    addr
    for word_addr in EXPECTED_UICR_WORDS
    for addr in range(word_addr, word_addr + 4)
}


def parse_ihex(path: Path):
    mem = {}
    upper = 0
    mode = "linear"
    segment = 0

    for n, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"{path}:{n}: invalid Intel HEX")

        rec = bytes.fromhex(line[1:])
        count, a_hi, a_lo, typ = rec[:4]
        data = rec[4:4 + count]

        if len(rec) != 5 + count:
            raise ValueError(f"{path}:{n}: malformed Intel HEX record")
        if (sum(rec) & 0xFF) != 0:
            raise ValueError(f"{path}:{n}: checksum mismatch")

        addr16 = (a_hi << 8) | a_lo

        if typ == 0:
            base = upper if mode == "linear" else segment
            for i, b in enumerate(data):
                addr = base + addr16 + i
                previous = mem.get(addr)
                if previous is not None and previous != b:
                    raise ValueError(
                        f"{path}:{n}: conflicting data at 0x{addr:08X}"
                    )
                mem[addr] = b
        elif typ == 1:
            break
        elif typ == 2:
            segment = int.from_bytes(data, "big") << 4
            mode = "segment"
        elif typ == 4:
            upper = int.from_bytes(data, "big") << 16
            mode = "linear"
        elif typ in (3, 5):
            # Start-segment/start-linear metadata, not programmed bytes.
            pass
        else:
            raise ValueError(
                f"{path}:{n}: unsupported HEX record type 0x{typ:02X}"
            )

    return mem


def read_le_word(mem, address):
    present = [address + i in mem for i in range(4)]
    if not any(present):
        return None
    if not all(present):
        missing = [address + i for i, ok in enumerate(present) if not ok]
        raise SystemExit(
            "ERROR: partial UICR word at "
            f"0x{address:08X}; missing "
            + ", ".join(f"0x{x:08X}" for x in missing)
        )
    return int.from_bytes(bytes(mem[address + i] for i in range(4)), "little")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hex", type=Path)
    args = ap.parse_args()

    mem = parse_ihex(args.hex)
    if not mem:
        raise SystemExit("ERROR: HEX contains no data")

    flash_addrs = [
        a for a in mem
        if BOOTLOADER_BASE <= a < BOOTLOADER_DATA_END
    ]
    uicr_addrs = [a for a in mem if a in ALLOWED_UICR_BYTES]
    unexpected = [
        a for a in mem
        if not (BOOTLOADER_BASE <= a < BOOTLOADER_DATA_END)
        and a not in ALLOWED_UICR_BYTES
    ]

    if not flash_addrs:
        raise SystemExit(
            f"ERROR: no bootloader flash data found at/above 0x{BOOTLOADER_BASE:08X}"
        )

    if unexpected:
        raise SystemExit(
            "ERROR: HEX contains data outside the permitted bootloader/UICR "
            f"regions; first unexpected address 0x{min(unexpected):08X}"
        )

    for address, expected in EXPECTED_UICR_WORDS.items():
        value = read_le_word(mem, address)
        if value is None:
            print(
                f"NOTE: UICR word 0x{address:08X} is not present in this HEX."
            )
            continue
        if value != expected:
            raise SystemExit(
                f"ERROR: UICR word 0x{address:08X} = 0x{value:08X}; "
                f"expected 0x{expected:08X}"
            )
        print(
            f"OK: UICR 0x{address:08X} = 0x{value:08X}"
        )

    flash_lo = min(flash_addrs)
    flash_hi = max(flash_addrs)

    if flash_lo != BOOTLOADER_BASE:
        raise SystemExit(
            f"ERROR: bootloader flash starts at 0x{flash_lo:08X}; "
            f"expected 0x{BOOTLOADER_BASE:08X}"
        )

    # Search only programmed flash bytes for the OpenPuck build marker. Sparse gaps are
    # filled with 0xFF so address ordering is preserved.
    blob = bytearray(b"\xFF" * (flash_hi - flash_lo + 1))
    for addr in flash_addrs:
        blob[addr - flash_lo] = mem[addr]

    print(
        f"OK: bootloader flash data range "
        f"0x{flash_lo:08X}..0x{flash_hi:08X}"
    )
    print(
        "OK: no SoftDevice/application/settings-page writes and no unexpected "
        "UICR writes"
    )

    if EXPECTED_UF2_MARKER not in bytes(blob):
        raise SystemExit(
            "ERROR: expected UF2 bootloader identity marker not found: "
            + EXPECTED_UF2_MARKER.decode("ascii")
        )
    print(
        "OK: UF2 bootloader identity marker found: "
        + EXPECTED_UF2_MARKER.decode("ascii")
    )


if __name__ == "__main__":
    main()
