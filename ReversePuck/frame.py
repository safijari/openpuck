"""frame.py -- the USB-CDC wire protocol shared with OpenController/deck_input.cpp.

    frame = [0xA5][0x5A][type][len][payload(len)][sum8(type,len,payload...)]

The reader resynchronizes on the 2-byte sync word, so any stray bytes (a boot banner, a serial
monitor) are skipped harmlessly. Keep this file in lock-step with deck_input.{h,cpp}.
"""
import struct

SYNC0 = 0xA5
SYNC1 = 0x5A

# Deck -> nRF
T_INPUT = 0x01
T_CONTROL = 0x02
T_REQ_STATUS = 0x03
# nRF -> Deck
T_STATUS = 0x10
T_TEXT = 0x1F
# nRF -> Deck haptic: [intensity u16][left u16][right u16][lgain u8][rgain u8] (Deck rumble cmd 0xEB)
T_HAPTIC = 0x11

# INPUT payload: buttons u32, lx ly rx ry s16, lt rt u8, lpx lpy rpx rpy s16, lpp rpp u16, axyz/gxyz s16.
# 4 + 8 + 2 + 8 + 4 + 12 = 38 bytes, little-endian. Matches applyInput() in deck_input.cpp.
_INPUT = struct.Struct("<I4hBB4hHH6h")
INPUT_LEN = _INPUT.size  # 38


def _sum8(type_, payload):
    s = (type_ + len(payload)) & 0xFF
    for b in payload:
        s = (s + b) & 0xFF
    return s


def build(type_, payload=b""):
    """Serialize one frame."""
    payload = bytes(payload)
    return bytes([SYNC0, SYNC1, type_, len(payload)]) + payload + bytes([_sum8(type_, payload)])


def build_input(st):
    """st: an object/dict with Triton fields -> an INPUT frame."""
    g = st.get if isinstance(st, dict) else (lambda k, d=0: getattr(st, k, d))
    payload = _INPUT.pack(
        g("buttons", 0) & 0xFFFFFFFF,
        g("lx", 0), g("ly", 0), g("rx", 0), g("ry", 0),
        g("lt", 0) & 0xFF, g("rt", 0) & 0xFF,
        g("lpx", 0), g("lpy", 0), g("rpx", 0), g("rpy", 0),
        g("lpp", 0) & 0xFFFF, g("rpp", 0) & 0xFFFF,
        g("ax", 0), g("ay", 0), g("az", 0), g("gx", 0), g("gy", 0), g("gz", 0),
    )
    return build(T_INPUT, payload)


def parse_haptic(payload):
    """Decode a T_HAPTIC payload -> dict for the Deck rumble cmd. Mirrors deck_input.cpp."""
    if len(payload) < 8:
        return None
    intensity, left, right = struct.unpack_from("<HHH", payload, 0)
    lgain, rgain = payload[6], payload[7]
    return {"intensity": intensity, "left": left, "right": right,
            "lgain": lgain, "rgain": rgain}


def build_set_forwarding(on):
    return build(T_CONTROL, bytes([0x01, 1 if on else 0]))


def build_clear_bond(slot):
    """Tell the firmware to un-bond (remove) slot `slot` -- the GUI "remove" affordance."""
    return build(T_CONTROL, bytes([0x02, slot & 0xFF]))


def build_req_status():
    return build(T_REQ_STATUS)


def parse_status(payload):
    """Decode a T_STATUS payload into a dict. Mirrors deckStatusTask() in deck_input.cpp.
    Only USED bonds are sent; each carries its firmware slot index (for removal)."""
    if len(payload) < 5:
        return None
    flags, link_slot, sess_ch, nbond_max, count = payload[:5]
    bonds = []
    off = 5
    for _ in range(count):
        if off + 26 > len(payload):
            break
        slot = payload[off]
        alive = payload[off + 1]
        puuid = struct.unpack_from("<I", payload, off + 2)[0]
        iuuid = struct.unpack_from("<I", payload, off + 6)[0]
        serial = payload[off + 10:off + 26].split(b"\x00")[0].decode("latin1", "replace")
        bonds.append({"slot": slot, "used": True, "alive": bool(alive),
                      "puuid": puuid, "iuuid": iuuid, "serial": serial})
        off += 26
    return {
        "forwarding": bool(flags & 1),
        "link_up": bool(flags & 2),
        "link_slot": None if link_slot == 0xFF else link_slot,
        "sess_ch": sess_ch,
        "nbond_max": nbond_max,
        "bonds": bonds,
    }


class Reader:
    """Feed raw bytes; yields (type, payload) tuples for each complete, checksum-valid frame.

    Bytes that fall *between* frames are the firmware's plain-text `#` diagnostic lines (Serial.printf,
    which the framed protocol resyncs past). We capture those too and surface them as (T_TEXT, line) so
    the app can show the firmware log without a serial monitor.
    """

    def __init__(self):
        self._st = 0
        self._type = 0
        self._len = 0
        self._buf = bytearray()
        self._sum = 0
        self._text = bytearray()

    def _flush_text(self, out):
        line = self._text.rstrip(b"\r\n")
        self._text = bytearray()
        if line and all(9 <= b < 127 or b in (10, 13) for b in line):
            out.append((T_TEXT, bytes(line)))

    def feed(self, data):
        out = []
        for c in data:
            st = self._st
            if st == 0:
                if c == SYNC0:
                    self._st = 1
                else:
                    # inter-frame byte: accumulate the firmware's plain-text diagnostic lines
                    self._text.append(c)
                    if c == 0x0A or len(self._text) > 240:
                        self._flush_text(out)
            elif st == 1:
                self._st = 2 if c == SYNC1 else (1 if c == SYNC0 else 0)
            elif st == 2:
                self._type = c
                self._sum = c
                self._st = 3
            elif st == 3:
                self._len = c
                self._sum = (self._sum + c) & 0xFF
                self._buf = bytearray()
                self._st = 5 if c == 0 else 4
            elif st == 4:
                self._buf.append(c)
                self._sum = (self._sum + c) & 0xFF
                if len(self._buf) >= self._len:
                    self._st = 5
            elif st == 5:
                if c == self._sum:
                    out.append((self._type, bytes(self._buf)))
                self._st = 0
        return out
