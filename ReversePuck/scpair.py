#!/usr/bin/env python3
"""scpair.py -- Linux hidraw steamless pairing for the Steam Controller 2 puck + (emulated) controller.

The Linux equivalent of pairtui/scmd.c (which is macOS/IOKit). Talks the Valve feature-report command
channel on the usagePage-0x01 control interface over /dev/hidraw* (HIDIOCSFEATURE / HIDIOCGFEATURE).
Used to pair on the Steam Deck itself when BOTH the puck (PID 1304) and the ReversePuck controller
(PID 1302) are plugged into the Deck -- or run it on any Linux box.

Pairing flow (protocol/USB_COMMANDS.md, == pairtui.pair_full):
  1. fresh 8-byte key (r1,r2)
  2. puck   slot N  (rid 2):  0xA2 [r1][r2][controller_serial 16]
  3. ctrl   (rid 1):          0xEE ["esb/bond"\\0][r1][r2][puck_serial 16]  then  0xEF ["esb/bond"\\0]
  4. ctrl   (rid 1):          0x95 52 AF 27 A4   (reboot into wireless)

    sudo python3 scpair.py list
    sudo python3 scpair.py pair [--slot N]
    sudo python3 scpair.py unpair --slot N
Needs read/write on /dev/hidraw* (run with sudo or install the udev rule in README.md).
"""
import argparse
import array
import fcntl
import glob
import os
import struct
import sys
import time

# ---- Linux ioctl numbers for hidraw (asm-generic _IOC) ----
_IOC_NONE, _IOC_WRITE, _IOC_READ = 0, 1, 2


def _IOC(d, t, nr, size):
    return (d << 30) | (size << 16) | (ord(t) << 8) | nr


def _IOR(t, nr, size):
    return _IOC(_IOC_READ, t, nr, size)


def _IOC_SF(length):
    return _IOC(_IOC_READ | _IOC_WRITE, 'H', 0x06, length)  # HIDIOCSFEATURE


def _IOC_GF(length):
    return _IOC(_IOC_READ | _IOC_WRITE, 'H', 0x07, length)  # HIDIOCGFEATURE


HIDIOCGRDESCSIZE = _IOR('H', 0x01, 4)
# HIDIOCGRDESC: struct hidraw_report_descriptor { __u32 size; __u8 value[4096]; }
HIDIOCGRDESC = _IOR('H', 0x02, 4 + 4096)

VALVE_VID = 0x28DE
PUCK_PID = 0x1304
CTRL_PIDS = (0x1302, 0x1301, 0x1303, 0x1205)
FEAT_LEN = 64  # Valve feature reports are exactly 64 bytes


def _sysfs_ids(node):
    """Return (vid, pid) for a /dev/hidrawN node, or (None, None)."""
    name = os.path.basename(node)
    try:
        # /sys/class/hidraw/hidrawN/device/uevent has HID_ID=0003:000028DE:00001304
        ue = open("/sys/class/hidraw/%s/device/uevent" % name).read()
    except Exception:
        return (None, None)
    for line in ue.splitlines():
        if line.startswith("HID_ID="):
            parts = line.split(":")
            try:
                return (int(parts[1], 16) & 0xFFFF, int(parts[2], 16) & 0xFFFF)
            except Exception:
                return (None, None)
    return (None, None)


def _first_usage_page(fd):
    """Parse the report descriptor's first Usage Page item. 0x01=generic desktop (control), 0xFF00=vendor."""
    try:
        size = array.array('i', [0])
        fcntl.ioctl(fd, HIDIOCGRDESCSIZE, size, True)
        buf = bytearray(4 + 4096)
        struct.pack_into("<I", buf, 0, size[0])
        fcntl.ioctl(fd, HIDIOCGRDESC, buf, True)
        n = struct.unpack_from("<I", buf, 0)[0]
        d = buf[4:4 + n]
        i = 0
        while i < len(d):
            b = d[i]
            if b == 0x05:  # Usage Page, 1-byte
                return d[i + 1]
            if b == 0x06:  # Usage Page, 2-byte
                return d[i + 1] | (d[i + 2] << 8)
            # skip item by its size (low 2 bits)
            i += 1 + (b & 0x03)
    except Exception:
        pass
    return None


class HidDev:
    def __init__(self, node, vid, pid):
        self.node = node
        self.vid = vid
        self.pid = pid
        self.fd = os.open(node, os.O_RDWR)
        self.usage_page = _first_usage_page(self.fd)

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def set_feature(self, rid, cmd, payload=b""):
        buf = bytearray(FEAT_LEN)
        buf[0] = rid
        buf[1] = cmd
        buf[2] = len(payload)
        buf[3:3 + len(payload)] = bytes(payload)
        fcntl.ioctl(self.fd, _IOC_SF(FEAT_LEN), buf, True)

    def get_feature(self, rid):
        buf = bytearray(FEAT_LEN)
        buf[0] = rid
        fcntl.ioctl(self.fd, _IOC_GF(FEAT_LEN), buf, True)
        return bytes(buf)

    def read_serial(self, rid):
        """0xAE idx 1 -> unit serial string."""
        try:
            self.set_feature(rid, 0xAE, bytes([0x01]))
            r = self.get_feature(rid)
            # reply [rid][AE][len][idx][serial...]
            return r[4:4 + 16].split(b"\x00")[0].decode("latin1", "replace")
        except Exception:
            return ""


def enumerate_devices():
    """Return dict role -> list[HidDev]: {'puck':[...], 'ctrl':[...]} (control interfaces only)."""
    out = {"puck": [], "ctrl": []}
    for node in sorted(glob.glob("/dev/hidraw*")):
        vid, pid = _sysfs_ids(node)
        if vid != VALVE_VID:
            continue
        try:
            dev = HidDev(node, vid, pid)
        except PermissionError:
            print("permission denied on %s (run with sudo or install the udev rule)" % node, file=sys.stderr)
            continue
        except Exception:
            continue
        # control interface only (usagePage 0x01); skip the vendor 0xFF00 diagnostic interface
        if dev.usage_page not in (0x01, None):
            dev.close()
            continue
        if pid == PUCK_PID:
            out["puck"].append(dev)
        elif pid in CTRL_PIDS:
            out["ctrl"].append(dev)
        else:
            dev.close()
    return out


def _ser16(serial):
    b = serial.encode("latin1", "replace")[:16]
    return b + b"\x00" * (16 - len(b))


def read_slots(puck):
    """Read the puck's 4 bond slots (each puck control interface == one slot). Returns list of dicts."""
    slots = []
    for i, dev in enumerate(sorted(puck, key=lambda d: d.node)):
        try:
            dev.set_feature(2, 0xA3)
            r = dev.get_feature(2)  # [02 A3 18 <8 uuid><16 serial>]
            rec = r[3:3 + 24]
            used = any(rec[0:8])
            serial = rec[8:24].split(b"\x00")[0].decode("latin1", "replace") if used else ""
            slots.append({"idx": i, "used": used, "serial": serial, "rec": rec})
        except Exception as ex:
            slots.append({"idx": i, "used": False, "serial": "", "rec": b"", "err": str(ex)})
    return slots


def pair(slot=None, pkey=None):
    devs = enumerate_devices()
    pucks = sorted(devs["puck"], key=lambda d: d.node)
    ctrls = sorted(devs["ctrl"], key=lambda d: d.node)
    if not pucks:
        raise SystemExit("no puck (28DE:1304) control interface found")
    if not ctrls:
        raise SystemExit("no controller (28DE:1302) control interface found — plug the ReversePuck controller in")
    ctrl = ctrls[0]
    puck_serial = pucks[0].read_serial(2) or "FXB000000000"
    ctrl_serial = ctrl.read_serial(1) or "FXA000000000"

    if slot is None:
        slots = read_slots(pucks)
        slot = next((s["idx"] for s in slots if not s["used"]), 0)
    if slot >= len(pucks):
        raise SystemExit("slot %d has no puck control interface" % slot)

    if pkey is None: 
        r = os.urandom(8)
    else: 
        r = pkey

    key = b"esb/bond\x00" if slot == 0 else b"esb/bond_2\x00"
    print("pairing puck %s slot %d <-> controller %s  key=%s" %
          (puck_serial, slot, ctrl_serial, r.hex()))

    # puck slot: 0xA2 [r1 r2][ctrl serial]
    pucks[slot].set_feature(2, 0xA2, r + _ser16(ctrl_serial))
    # controller esb/bond: 0xEE [key\0][r1 r2][puck serial] + 0xEF [key\0]
    ctrl.set_feature(1, 0xEE, key + r + _ser16(puck_serial))
    ctrl.set_feature(1, 0xEF, key)
    time.sleep(0.1)
    # reboot controller into wireless mode (magic 0xA427AF52)
    ctrl.set_feature(1, 0x95, bytes([0x52, 0xAF, 0x27, 0xA4]))
    print("paired. controller rebooting into wireless — move the puck to your host.")

def write_puck_slot(slot=None, ctrl_serial=None, pkey=None):
    devs = enumerate_devices()
    pucks = sorted(devs["puck"], key=lambda d: d.node)
    if not pucks:
        raise SystemExit("no puck (28DE:1304) control interface found")

    if slot is None:
        slots = read_slots(pucks)
        slot = next((s["idx"] for s in slots if not s["used"]), 0)
    if slot >= len(pucks):
        raise SystemExit("slot %d has no puck control interface" % slot)

    
    if ctrl_serial is None and pkey is None: 
        print("wiping puck slot %d" % (slot))
        pucks[slot].set_feature(2, 0xA2, b"\x00" * 24)
    else:
        if pkey is None: 
            raise SystemExit("No key provided!")
        
        print("writing to puck slot %d: controller %s key=%s" %
          (slot, ctrl_serial, pkey.hex()))
        # puck slot: 0xA2 [r1 r2][ctrl serial]
        pucks[slot].set_feature(2, 0xA2, pkey + _ser16(ctrl_serial))

    print("done")

def write_controller_slot(slot=0, puck_serial=None, pkey=None):
    devs = enumerate_devices()
    ctrls = sorted(devs["ctrl"], key=lambda d: d.node)
    if not ctrls:
        raise SystemExit("no controller (28DE:1302) control interface found — plug the ReversePuck controller in")
    
    if slot not in [0, 1]:
        raise SystemExit("invalid controller slot %d" % (slot))


    ctrl = ctrls[0]

    if pkey is None: 
        r = os.urandom(8)
    else: 
        r = pkey

    key = b"esb/bond\x00" if slot == 0 else b"esb/bond_2\x00"

    if puck_serial is None and pkey is None: 
        print("Wiping controller slot %d" % (slot))
        try:
            ctrl.set_feature(1, 0xEE, key + b"\x00" * 24)
            ctrl.set_feature(1, 0xEF, key)
            print("Slot wiped.")
        except Exception:
            print("Slot wipe failed.")
        
    else: 

        print("Writing controller slot %d: puck serial %s key=%s" %
          (slot, puck_serial, r.hex()))

        # controller esb/bond: 0xEE [key\0][r1 r2][puck serial] + 0xEF [key\0]
        ctrl.set_feature(1, 0xEE, key + r + _ser16(puck_serial))
        ctrl.set_feature(1, 0xEF, key)
        time.sleep(0.1)
        # reboot controller into wireless mode (magic 0xA427AF52)
        ctrl.set_feature(1, 0x95, bytes([0x52, 0xAF, 0x27, 0xA4]))
        print("paired. controller rebooting into wireless — move the puck to your host.")


def unpair(slot):
    devs = enumerate_devices()
    pucks = sorted(devs["puck"], key=lambda d: d.node)
    if slot >= len(pucks):
        raise SystemExit("slot %d has no puck control interface" % slot)
    pucks[slot].set_feature(2, 0xA2, b"\x00" * 24)  # clear puck slot
    # also clear the controller bond if present
    for ctrl in devs["ctrl"]:
        key = b"esb/bond\x00" if slot == 0 else b"esb/bond_2\x00"
        try:
            ctrl.set_feature(1, 0xEE, key + b"\x00" * 24)
            ctrl.set_feature(1, 0xEF, key)
        except Exception:
            print("Slot wipe on controller failed.")
    print("cleared slot %d" % slot)



def cmd_list():
    devs = enumerate_devices()
    pucks = sorted(devs["puck"], key=lambda d: d.node)
    ctrls = sorted(devs["ctrl"], key=lambda d: d.node)
    print("controllers:", [(d.node, "%04X" % d.pid, d.read_serial(1)) for d in ctrls] or "(none)")
    if pucks:
        print("puck:", pucks[0].read_serial(2))
        for s in read_slots(pucks):
            if s["used"]:
                print("  slot %d: %s (key %s)" % (s["idx"], s["serial"], s["rec"][:8].hex()))
            else: 
                print("  slot %d: Empty" % (s["idx"]))
    else:
        print("puck: (none)")


def steam_key_check(key):
    try: 
        key = bytes.fromhex(key)
        if len(key) == 8: 
            return key
    except: 
        raise argparse.ArgumentTypeError("Invalid pairing key!")


def main():
    ap = argparse.ArgumentParser(description="Linux hidraw steamless pairing")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("pair")
    p.add_argument("--slot", type=int, default=None)
    p.add_argument("--key", type=steam_key_check, default=None)
    u = sub.add_parser("unpair")
    u.add_argument("--slot", type=int, required=True)

    s = sub.add_parser("write-puck")
    s.add_argument("--slot", type=int, default=None)
    s.add_argument("--serial", type=str, default=None)
    s.add_argument("--key", type=steam_key_check, default=None)

    c = sub.add_parser("write-controller")
    c.add_argument("--slot", type=int, default=None)
    c.add_argument("--serial", type=str, default=None)
    c.add_argument("--key", type=steam_key_check, default=None)


    
    args = ap.parse_args()
    if args.cmd == "list":
        cmd_list()
    elif args.cmd == "pair":
        pair(args.slot, args.key)
    elif args.cmd == "unpair":
        unpair(args.slot)
    elif args.cmd == "write-puck":
        write_puck_slot(args.slot, args.serial, args.key)
    elif args.cmd == "write-controller":
        write_controller_slot(args.slot, args.serial, args.key)



if __name__ == "__main__":
    main()
