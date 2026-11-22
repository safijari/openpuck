#!/usr/bin/env python3
"""Draw the ReversePuck AppImage icon (a puck with radio waves) as a 256x256 PNG, stdlib only.

    python3 make_icon.py OUT.png
"""
import math
import struct
import sys
import zlib

SIZE = 256
BG = (0x1B, 0x28, 0x38)
PUCK = (0xE8, 0xEC, 0xF1)
WAVE = (0x66, 0xC0, 0xF4)


def pixel(x, y):
    """RGBA at pixel centre (x, y): rounded-square tile, puck at lower left, waves to the upper right."""
    px, py = x + 0.5, y + 0.5
    # rounded-square tile with 40px corners
    r = 40
    dx = max(r - px, 0, px - (SIZE - r))
    dy = max(r - py, 0, py - (SIZE - r))
    if dx * dx + dy * dy > r * r:
        return (0, 0, 0, 0)
    cx, cy = 92, 164
    d = math.hypot(px - cx, py - cy)
    if d <= 44:
        return PUCK + (255,)
    # three arcs centred on the puck, only in its upper-right quarter
    ang = math.degrees(math.atan2(cy - py, px - cx))
    if 5 <= ang <= 85:
        for radius in (78, 112, 146):
            if abs(d - radius) <= 9:
                return WAVE + (255,)
    return BG + (255,)


def png(rows):
    raw = b"".join(b"\x00" + bytes(c for p in row for c in p) for row in rows)

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", SIZE, SIZE, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9))
            + chunk(b"IEND", b""))


if __name__ == "__main__":
    with open(sys.argv[1], "wb") as f:
        f.write(png([[pixel(x, y) for x in range(SIZE)] for y in range(SIZE)]))
