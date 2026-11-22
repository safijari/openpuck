#!/usr/bin/env python3
"""
Create a one-time UF2 that installs:
  - Nordic S140 6.1.1 / MBR below 0x26000
  - an OpenPuck application linked for 0x26000..0xECFFF

It deliberately does NOT include:
  - 0xED000..0xF3FFF (OpenPuck InternalFS)
  - 0xF4000..0xFFFFF (Makerdiary UF2 bootloader/config/settings)

This is intended for Makerdiary/GeeekPi nRF52840 MDK USB Dongles whose
factory INFO_UF2.TXT reports S132 5.1.0. The existing UF2 bootloader remains
the recovery mechanism.

The input application must already be compiled with:
  - FQBN adafruit:nrf52:feather52840
  - -DOPK_BOARD_MDK_USB_DONGLE=1
  - OpenPuck's required TinyUSB flags
"""

from __future__ import annotations
import argparse
import struct
from pathlib import Path

UF2_MAGIC_START0 = 0x0A324655
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
NRF52840_FAMILY_ID = 0xADA52840

APP_BASE = 0x26000
APP_END = 0xED000
BOOTLOADER_BASE = 0xF4000
PAYLOAD_SIZE = 256


def parse_ihex(path: Path) -> dict[int, int]:
    mem: dict[int, int] = {}
    linear_base = 0
    segment_base = 0
    mode = "linear"

    for lineno, raw in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"{path}:{lineno}: not Intel HEX")
        rec = bytes.fromhex(line[1:])
        if len(rec) < 5:
            raise ValueError(f"{path}:{lineno}: short record")

        count = rec[0]
        addr16 = (rec[1] << 8) | rec[2]
        rectype = rec[3]
        data = rec[4:4 + count]
        checksum = rec[4 + count]

        if (sum(rec[:5 + count]) & 0xFF) != 0:
            raise ValueError(f"{path}:{lineno}: checksum mismatch")

        if rectype == 0x00:
            base = linear_base if mode == "linear" else segment_base
            addr = base + addr16
            for b in data:
                old = mem.get(addr)
                if old is not None and old != b:
                    raise ValueError(f"{path}:{lineno}: conflicting byte at 0x{addr:08X}")
                mem[addr] = b
                addr += 1
        elif rectype == 0x01:
            break
        elif rectype == 0x02:
            segment_base = int.from_bytes(data, "big") << 4
            mode = "segment"
        elif rectype == 0x04:
            linear_base = int.from_bytes(data, "big") << 16
            mode = "linear"
        elif rectype in (0x03, 0x05):
            # Start-address metadata; not flash contents.
            pass
        else:
            raise ValueError(f"{path}:{lineno}: unsupported record type 0x{rectype:02X}")
    return mem


def pages_from_memory(mem: dict[int, int]) -> dict[int, bytearray]:
    pages: dict[int, bytearray] = {}
    for addr, value in mem.items():
        base = addr & ~(PAYLOAD_SIZE - 1)
        page = pages.setdefault(base, bytearray(b"\xFF" * PAYLOAD_SIZE))
        page[addr - base] = value
    return pages


def encode_uf2(pages: dict[int, bytearray]) -> bytes:
    addresses = sorted(pages)
    total = len(addresses)
    blocks = []
    for block_no, addr in enumerate(addresses):
        data = bytes(pages[addr])
        header = struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            UF2_FLAG_FAMILY_ID_PRESENT,
            addr,
            PAYLOAD_SIZE,
            block_no,
            total,
            NRF52840_FAMILY_ID,
        )
        padding = b"\x00" * (512 - 32 - PAYLOAD_SIZE - 4)
        blocks.append(header + data + padding + struct.pack("<I", UF2_MAGIC_END))
    return b"".join(blocks)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--softdevice", type=Path, required=True)
    ap.add_argument("--application", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()

    sd = parse_ihex(args.softdevice)
    app = parse_ihex(args.application)

    if not sd:
        raise SystemExit("SoftDevice HEX contains no flash data")
    if not app:
        raise SystemExit("Application HEX contains no flash data")

    sd_min, sd_max = min(sd), max(sd)
    app_min, app_max = min(app), max(app)

    # S140 6.1.1 must stay entirely below the Adafruit application base.
    if sd_min != 0:
        raise SystemExit(f"SoftDevice/MBR does not begin at 0x000000 (got 0x{sd_min:06X})")
    if sd_max >= APP_BASE:
        raise SystemExit(
            f"SoftDevice writes at/above 0x{APP_BASE:06X} (highest 0x{sd_max:06X}); refusing"
        )

    # The compiled OpenPuck image must be a pure application using the standard
    # S140/Adafruit layout. Reject accidental SoftDevice/bootloader-containing HEXes.
    if app_min < APP_BASE:
        raise SystemExit(
            f"Application writes below 0x{APP_BASE:06X} (lowest 0x{app_min:06X}); refusing"
        )
    if app_max >= APP_END:
        raise SystemExit(
            f"Application reaches 0x{app_max:06X}, outside OpenPuck app region ending "
            f"at 0x{APP_END - 1:06X}; refusing"
        )

    overlap = set(sd).intersection(app)
    if overlap:
        first = min(overlap)
        raise SystemExit(f"SoftDevice/application overlap at 0x{first:06X}; refusing")

    merged = dict(sd)
    merged.update(app)
    pages = pages_from_memory(merged)

    if any(addr >= BOOTLOADER_BASE for addr in pages):
        raise SystemExit("Generated image would touch the Makerdiary bootloader; refusing")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    uf2 = encode_uf2(pages)
    args.output.write_bytes(uf2)

    print(f"Wrote {args.output}")
    print(f"UF2 blocks: {len(pages)}")
    print(f"S140/MBR bytes: 0x{sd_min:06X}..0x{sd_max:06X}")
    print(f"OpenPuck bytes: 0x{app_min:06X}..0x{app_max:06X}")
    print(f"Protected bootloader starts at: 0x{BOOTLOADER_BASE:06X}")


if __name__ == "__main__":
    main()
