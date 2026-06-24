"""input_source.py -- read the Steam Deck's built-in controls and normalize to Triton fields.

TWO sources:

* **DeckSource (default, libusb/pyusb)** -- the CORRECT way, modeled on ~/Work/joverlay. The Deck
  controller is USB 28DE:1205; Steam reads its 64-byte "DeckState" vendor report on interface 2,
  EP 0x83. We **detach the kernel driver from interfaces 0/1/2 and claim them** -> the controller
  vanishes from the kernel/Steam (true USB-level detach, so Steam stops eating Steam/QAM), and we read
  EP 0x83 ourselves and decode the raw report (trackpads, sticks, triggers, gyro, ALL buttons incl.
  Steam/QAM). Release + reattach hands it back to Steam. Needs access to the usbfs node (see setup.sh:
  `chmod` the /dev/bus/usb node, or run as root).

* **EvdevSource (fallback, --evdev)** -- pure-stdlib evdev. Works without libusb but the Deck exposes
  trackpads as sticks/dpad and can't see Steam/QAM, hence the original mapping problems.

DeckState layout: buttons in bytes 8-19 (the bits below are from joverlay's live captures + hid-steam.c /
SDL_hidapi_steamdeck.c); analog (pads/sticks/triggers/IMU) at byte 20+. Run `python3 input_source.py`
to live-dump the raw report and verify/correct the offsets on your unit.
"""
from __future__ import annotations
import fcntl
import glob
import os
import struct

# ---- Triton button masks (mirror triton.h) ----
TB_A = 0x1
TB_B = 0x2
TB_X = 0x4
TB_Y = 0x8
TB_QAM = 0x10
TB_R3 = 0x20
TB_VIEW = 0x40
TB_R4 = 0x80
TB_R5 = 0x100
TB_RB = 0x200
TB_DDN = 0x400
TB_DRT = 0x800
TB_DLF = 0x1000
TB_DUP = 0x2000
TB_MENU = 0x4000
TB_L3 = 0x8000
TB_STEAM = 0x10000
TB_L4 = 0x20000
TB_L5 = 0x40000
TB_LB = 0x80000

NEUTRAL = {
    "buttons": 0, "lx": 0, "ly": 0, "rx": 0, "ry": 0, "lt": 0, "rt": 0,
    "lpx": 0, "lpy": 0, "rpx": 0, "rpy": 0, "lpp": 0, "rpp": 0,
    "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0,
}

DECK_VID = 0x28DE
DECK_PID = 0x1205

# ─────────────────────────── DeckState decode ───────────────────────────
# Each entry: (byte_offset, bit_mask, Triton mask). EXACT byte/bit from the Linux kernel
# drivers/hid/hid-steam.c steam_do_deck_input_event() -- authoritative, not guessed.
DECK_BTN_MAP = [
    # byte 8 -- digital triggers (full-pull) + bumpers + face buttons
    (8, 0x01, 0x00800000),  # R2 full-pull -> RTrig click (triton bit 23)
    (8, 0x02, 0x08000000),  # L2 full-pull -> LTrig click (triton bit 27)
    (8, 0x04, TB_RB),       # R1
    (8, 0x08, TB_LB),       # L1
    (8, 0x10, TB_Y),
    (8, 0x20, TB_B),
    (8, 0x40, TB_X),
    (8, 0x80, TB_A),
    # byte 9 -- dpad + system buttons + lower-left grip
    (9, 0x01, TB_DUP),
    (9, 0x02, TB_DRT),
    (9, 0x04, TB_DLF),
    (9, 0x08, TB_DDN),
    # SELECT (byte9 bit4) and START (byte9 bit6) are SWAPPED vs the Triton View/Menu so they read
    # right on the host (kernel calls bit4 "menu left"/SELECT, bit6 "menu right"/START).
    (9, 0x10, TB_MENU),    # ⊟  (BTN_SELECT)  -> Triton Menu
    (9, 0x20, TB_STEAM),   # Steam (BTN_MODE)
    (9, 0x40, TB_VIEW),    # ☰  (BTN_START)   -> Triton View
    (9, 0x80, TB_L5),      # GRIPL2 -- lower-left back paddle
    # byte 10 -- lower-right grip + pad click/TOUCH + left stick click
    (10, 0x01, TB_R5),         # GRIPR2 -- lower-right back paddle
    (10, 0x02, 0x04000000),    # BTN_THUMB  -> left pad CLICK  (TB_LPADC)
    (10, 0x04, 0x00400000),    # BTN_THUMB2 -> right pad CLICK (TB_RPADC)
    (10, 0x08, 0x02000000),    # left pad TOUCH  (TB_LPADT bit25) -- needed for pad->mouse
    (10, 0x10, 0x00200000),    # right pad TOUCH (TB_RPADT bit21)
    (10, 0x40, TB_L3),         # BTN_THUMBL -> left stick click
    # byte 11 -- right stick click
    (11, 0x04, TB_R3),         # BTN_THUMBR
    # byte 13 -- upper back paddles + thumbstick capacitive touch
    (13, 0x02, TB_L4),         # BTN_GRIPL  (upper-left)
    (13, 0x04, TB_R4),         # BTN_GRIPR  (upper-right)
    (13, 0x40, 0x01000000),    # left thumbstick TOUCH  (LJoy touch, bit24)
    (13, 0x80, 0x00100000),    # right thumbstick TOUCH (RJoy touch, bit20)
    # byte 14 -- QAM (•••)
    (14, 0x04, TB_QAM),        # BTN_BASE
]

# analog block: little-endian 16-bit, EXACT offsets from hid-steam.c steam_do_deck_input_event().
_OFF_LPAD_X, _OFF_LPAD_Y = 16, 18   # s16 left trackpad
_OFF_RPAD_X, _OFF_RPAD_Y = 20, 22   # s16 right trackpad
_OFF_ACCEL = 24                     # s16 ax,ay,az (24/26/28)
_OFF_GYRO = 30                      # s16 gx,gy,gz (30/32/34)
_OFF_TRIG_L, _OFF_TRIG_R = 44, 46   # u16 analog triggers
_OFF_LSTICK_X, _OFF_LSTICK_Y = 48, 50
_OFF_RSTICK_X, _OFF_RSTICK_Y = 52, 54
_OFF_LPAD_P, _OFF_RPAD_P = 56, 58   # u16 trackpad pressure

# ── IMU axis remap: Deck IMU -> SC2 report-0x45 IMU ──────────────────────────
# The Deck and the SC2 mount their IMUs in different orientations, so the raw Deck axes do NOT line up
# with the gyro/accel convention Steam expects from an SC2. accel and gyro MUST use the SAME remap (shared
# handedness) or the host's sensor fusion mixes axes intermittently (see memory: switch-imu-fusion).
# Each entry = (source_axis, sign): source_axis 0=x 1=y 2=z indexes the raw Deck (x,y,z); sign flips it.
# CALIBRATE empirically: in Steam's gyro test, rotate the Deck about one physical axis, see which on-screen
# axis moves and which way, then swap the source / flip the sign here. Default = identity pass-through.
IMU_GYRO_MAP = ((0, +1), (1, +1), (2, +1))    # -> (gx, gy, gz)
IMU_ACCEL_MAP = ((0, +1), (1, +1), (2, +1))   # -> (ax, ay, az)
# Deck->SC2 gyro sensitivity ratio. Both are int16 IMUs at similar full-scale, so 1.0 is the start point;
# bump if the on-screen rotation is too slow, drop it if too fast/jittery.
IMU_GYRO_SCALE = 1.0


def _clamp16(v):
    return -32768 if v < -32768 else 32767 if v > 32767 else int(v)


def _remap3(src, mapping, scale=1.0):
    """src = (x, y, z) raw; mapping = 3x (source_axis, sign). Returns the remapped, clamped triple."""
    return tuple(_clamp16(src[ax] * sign * scale) for ax, sign in mapping)


def _s16(b, off):
    return struct.unpack_from("<h", b, off)[0]


def _u16(b, off):
    return struct.unpack_from("<H", b, off)[0]


def decode_deck(rpt, state):
    """Decode a 64-byte DeckState report (rpt) into the Triton `state` dict. Returns False if not a
    0x09 state report."""
    if len(rpt) < 60 or rpt[2] != 0x09:
        return False
    btn = 0
    for off, mask, tmask in DECK_BTN_MAP:
        if tmask and (rpt[off] & mask):
            btn |= tmask
    state["buttons"] = btn
    # sticks (int16, center 0). Deck Y is up-positive; Triton expects up-positive too -> pass through.
    state["lx"] = _s16(rpt, _OFF_LSTICK_X)
    state["ly"] = _s16(rpt, _OFF_LSTICK_Y)
    state["rx"] = _s16(rpt, _OFF_RSTICK_X)
    state["ry"] = _s16(rpt, _OFF_RSTICK_Y)
    # analog triggers u16 -> 0..255
    state["lt"] = min(255, _u16(rpt, _OFF_TRIG_L) >> 7)
    state["rt"] = min(255, _u16(rpt, _OFF_TRIG_R) >> 7)
    # trackpads (int16) -> Triton pads (this is the FIX: pads no longer bleed into sticks/dpad)
    state["lpx"] = _s16(rpt, _OFF_LPAD_X)
    state["lpy"] = _s16(rpt, _OFF_LPAD_Y)
    state["rpx"] = _s16(rpt, _OFF_RPAD_X)
    state["rpy"] = _s16(rpt, _OFF_RPAD_Y)
    state["lpp"] = _u16(rpt, _OFF_LPAD_P)   # trackpad pressure (u16)
    state["rpp"] = _u16(rpt, _OFF_RPAD_P)
    # IMU: raw Deck accel/gyro -> SC2 axes via the shared remap (calibrate IMU_*_MAP in Steam's gyro test).
    araw = (_s16(rpt, _OFF_ACCEL), _s16(rpt, _OFF_ACCEL + 2), _s16(rpt, _OFF_ACCEL + 4))
    graw = (_s16(rpt, _OFF_GYRO), _s16(rpt, _OFF_GYRO + 2), _s16(rpt, _OFF_GYRO + 4))
    state["ax"], state["ay"], state["az"] = _remap3(araw, IMU_ACCEL_MAP)
    state["gx"], state["gy"], state["gz"] = _remap3(graw, IMU_GYRO_MAP, IMU_GYRO_SCALE)
    return True


# ─────────────────────────── USB source (default) ───────────────────────────
class DeckSource:
    """Read + DETACH the Deck controller via libusb (pyusb). grab() detaches it from the kernel/Steam
    and starts reading EP 0x83; ungrab() hands it back."""

    IFACES = (0, 1, 2)   # claim all HID interfaces so the pad fully leaves the kernel (mouse/kbd/steam)
    EP_IN = 0x83
    IFACE_READ = 2

    def __init__(self, path=None):
        import time as _time
        import usb.core  # imported here so --help / --evdev work without pyusb
        self._time = _time
        self._usbutil = __import__("usb.util", fromlist=["util"])
        self.dev = usb.core.find(idVendor=DECK_VID, idProduct=DECK_PID)
        if self.dev is None:
            raise RuntimeError("Deck controller 28DE:1205 not found on USB")
        self.state = dict(NEUTRAL)
        self._claimed = False
        self._USBError = usb.core.USBError
        self._last_rumble = None      # last (int,left,right,lg,rg) we sent -> dedupe
        self._last_rumble_t = 0.0     # throttle clock

    @property
    def name(self):
        return "Steam Deck controller (libusb 28DE:1205)"

    def grab(self):
        if self._claimed:
            return
        for i in self.IFACES:
            try:
                if self.dev.is_kernel_driver_active(i):
                    self.dev.detach_kernel_driver(i)
            except Exception:
                pass
            try:
                self._usbutil.claim_interface(self.dev, i)
            except Exception:
                pass
        self._claimed = True

    def ungrab(self):
        if not self._claimed:
            return
        for i in self.IFACES:
            try:
                self._usbutil.release_interface(self.dev, i)
            except Exception:
                pass
            try:
                self.dev.attach_kernel_driver(i)
            except Exception:
                pass
        self._claimed = False

    def pump(self):
        # drain whatever reports are queued; keep the freshest decoded state
        for _ in range(8):
            try:
                rpt = self.dev.read(self.EP_IN, 64, timeout=2)
            except self._USBError:
                break  # timeout / no data
            decode_deck(bytes(rpt), self.state)
        return self.state

    # don't issue rumble feature-reports faster than this -- protects the Deck pad's control endpoint
    # (hammering it at the poll rate is what wedged the pad). A repeated identical state is also dropped.
    _RUMBLE_MIN_DT = 0.015

    def rumble(self, intensity, left, right, lgain=0, rgain=0):
        """Play a rumble on the Deck's haptics via the 0xEB feature report (hid-steam SET_REPORT).
        intensity/left/right are the SC2 rumble u16s relayed back from Steam; 0 stops it. Rate-limited
        + deduped + short-timeout so a rumble burst can never flood or block the input loop."""
        if not self._claimed:
            return
        cur = (intensity & 0xFFFF, left & 0xFFFF, right & 0xFFFF, lgain & 0xFF, rgain & 0xFF)
        now = self._time.monotonic()
        # always let an OFF (all-zero) through promptly; otherwise dedupe + throttle
        if cur != (0, 0, 0, 0, 0):
            if cur == self._last_rumble:
                return
            if now - self._last_rumble_t < self._RUMBLE_MIN_DT:
                return
        self._last_rumble = cur
        self._last_rumble_t = now
        buf = bytearray(64)
        buf[0] = 0xEB          # CMD_TRIGGER_HAPTIC_RUMBLE
        buf[1] = 0x09          # payload length
        buf[2] = 0x00          # reserved
        struct.pack_into("<HHH", buf, 3, cur[0], cur[1], cur[2])
        buf[9] = cur[3]
        buf[10] = cur[4]
        try:
            # SET_REPORT(Feature), report id 0, on the DeckState control interface (iface 2). Short
            # timeout: never block the input/UI loop if the endpoint is busy.
            self.dev.ctrl_transfer(0x21, 0x09, 0x0300, self.IFACE_READ, bytes(buf), timeout=20)
        except Exception:
            pass


# ─────────────────────────── evdev fallback ───────────────────────────
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR = 0x136, 0x137
BTN_SELECT, BTN_START, BTN_MODE = 0x13A, 0x13B, 0x13C
BTN_THUMBL, BTN_THUMBR = 0x13D, 0x13E
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0, 1, 2, 3, 4, 5
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11
KEYMAP = {
    BTN_SOUTH: TB_A, BTN_EAST: TB_B, BTN_WEST: TB_X, BTN_NORTH: TB_Y,
    BTN_TL: TB_LB, BTN_TR: TB_RB,
    BTN_SELECT: TB_VIEW, BTN_START: TB_MENU, BTN_MODE: TB_STEAM,
    BTN_THUMBL: TB_L3, BTN_THUMBR: TB_R3,
}
_EVENT = struct.Struct("qqHHi")
EVENT_SZ = _EVENT.size


def _IOC(d, t, nr, size):
    return (d << 30) | (size << 16) | (ord(t) << 8) | nr


EVIOCGID = _IOC(2, 'E', 0x02, 8)


def EVIOCGNAME(length):
    return _IOC(2, 'E', 0x06, length)


def EVIOCGBIT(ev, length):
    return _IOC(2, 'E', 0x20 + ev, length)


EVIOCGRAB = _IOC(1, 'E', 0x90, 4)


def _test_bit(bm, bit):
    return bool(bm[bit // 8] & (1 << (bit % 8)))


class _RawDev:
    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def ids(self):
        buf = bytearray(8)
        try:
            fcntl.ioctl(self.fd, EVIOCGID, buf, True)
            _b, vid, pid, _v = struct.unpack("<HHHH", buf)
            return vid, pid
        except Exception:
            return (0, 0)

    def has_key(self, code):
        buf = bytearray(96)
        try:
            fcntl.ioctl(self.fd, EVIOCGBIT(EV_KEY, len(buf)), buf, True)
            return _test_bit(buf, code)
        except Exception:
            return False

    def grab(self):
        fcntl.ioctl(self.fd, EVIOCGRAB, 1)

    def ungrab(self):
        try:
            fcntl.ioctl(self.fd, EVIOCGRAB, 0)
        except Exception:
            pass

    def read_events(self):
        try:
            data = os.read(self.fd, EVENT_SZ * 64)
        except (BlockingIOError, OSError):
            return
        for i in range(0, len(data) - EVENT_SZ + 1, EVENT_SZ):
            _s, _u, typ, code, val = _EVENT.unpack_from(data, i)
            yield typ, code, val


def find_deck_evdev():
    for path in sorted(glob.glob("/dev/input/event*")):
        try:
            d = _RawDev(path)
        except Exception:
            continue
        try:
            if d.has_key(BTN_SOUTH):
                return path
        finally:
            d.close()
    return None


class EvdevSource:
    def __init__(self, path=None):
        self.path = path or find_deck_evdev()
        if not self.path:
            raise RuntimeError("no gamepad evdev device found")
        self.dev = _RawDev(self.path)
        self.state = dict(NEUTRAL)
        self._grabbed = False

    @property
    def name(self):
        return "evdev " + self.path

    def grab(self):
        if not self._grabbed:
            self.dev.grab()
            self._grabbed = True

    def ungrab(self):
        if self._grabbed:
            self.dev.ungrab()
            self._grabbed = False

    def rumble(self, intensity, left, right, lgain=0, rgain=0):
        # evdev fallback has no haptic path (FF could be added, but the libusb source is the real one)
        pass

    def pump(self):
        s = self.state
        for typ, code, val in self.dev.read_events():
            if typ == EV_KEY:
                m = KEYMAP.get(code)
                if m is not None:
                    s["buttons"] = (s["buttons"] | m) if val else (s["buttons"] & ~m)
            elif typ == EV_ABS:
                if code == ABS_X:
                    s["lx"] = val
                elif code == ABS_Y:
                    s["ly"] = -val
                elif code == ABS_RX:
                    s["rx"] = val
                elif code == ABS_RY:
                    s["ry"] = -val
                elif code == ABS_Z:
                    s["lt"] = max(0, min(255, val))
                elif code == ABS_RZ:
                    s["rt"] = max(0, min(255, val))
        return s


def open_source(prefer_evdev=False, path=None):
    """Pick the best input source: libusb DeckSource by default, evdev if requested/unavailable."""
    if not prefer_evdev:
        try:
            return DeckSource(path)
        except Exception as ex:
            print("DeckSource (libusb) unavailable (%s); falling back to evdev" % ex)
    return EvdevSource(path)


# ─────────────────────────── capture / calibration tool ───────────────────────────
def _capture():
    """Live-dump the raw DeckState report; press one control at a time and watch which bytes/bits move,
    so you can correct DECK_BTN_MAP and the analog offsets for your unit. Ctrl-C to stop."""
    src = DeckSource()
    print("detaching controller (libusb) ... press ONE control at a time; Ctrl-C to stop")
    src.grab()
    base = None
    try:
        while True:
            try:
                rpt = bytes(src.dev.read(src.EP_IN, 64, timeout=200))
            except Exception:
                continue
            if len(rpt) < 60 or rpt[2] != 0x09:
                continue
            if base is None:
                base = rpt
                print("baseline:", base.hex())
                continue
            diff = [(i, base[i], rpt[i]) for i in range(8, 60) if base[i] != rpt[i]]
            if diff:
                print(" ".join("b%d:%02x->%02x" % d for d in diff))
    except KeyboardInterrupt:
        pass
    finally:
        src.ungrab()
        print("\nreattached controller to the kernel.")


if __name__ == "__main__":
    _capture()
