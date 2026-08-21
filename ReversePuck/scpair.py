#!/usr/bin/env python3
"""scpair.py -- Linux hidraw steamless pairing for the Steam Controller 2 puck + (emulated) controller.

Original code by safijari
Steam Machine, slot cloning and wireless controller support, firmware version query by Leseratte10.

The Linux equivalent of pairtui/scmd.c (which is macOS/IOKit). Talks the Valve feature-report command
channel on the usagePage-0x01 control interface over /dev/hidraw* (HIDIOCSFEATURE / HIDIOCGFEATURE).
Used to pair on the Steam Deck itself when BOTH the puck (PID 1304) and the ReversePuck controller
(PID 1302) are plugged into the Deck -- or run it on any Linux box.

This code is not limited to the ReversePuck, it can also be used to manage, reset, pair and clone
normal pairings between a Puck (or OpenPuck) and a Steam Controller.

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
from pathlib import Path

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
PUCK_PIDS = (0x1304, 0x1305)
CTRL_PIDS = (0x1302, 0x1301, 0x1303, 0x1205)


def _sysfs_ids(node):
    """Return (vid, pid) for a /dev/hidrawN node, or (None, None)."""
    name = os.path.basename(node)
    try:
        # /sys/class/hidraw/hidrawN/device/uevent has HID_ID=0003:000028DE:00001304
        ue = open("/sys/class/hidraw/%s/device/uevent" % name).read()
    except Exception:
        return (None, None)
    
    vid = None
    pid = None 

    for line in ue.splitlines():
        if line.startswith("HID_ID="):
            parts = line.split(":")
            try:
                vid = int(parts[1], 16) & 0xFFFF
                pid = int(parts[2], 16) & 0xFFFF
                return (vid, pid)
            except:
                pass

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
    def __init__(self, node, vid, pid, info_str = "Valve"):
        self.node = node
        self.vid = vid
        self.pid = pid
        self.info_str = info_str
        self.fd = os.open(node, os.O_RDWR)
        self.usage_page = _first_usage_page(self.fd)

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def get_bcd_version(self):
        """
        Relevant for Pucks. Both the standalone Puck and the receiver in the Steam Machine
        use bcdVersion 2, but Steam also supports Pucks with other versions that sometimes 
        behave differently.
        """
        dxi = self.node.replace('/dev/', '')
        base = Path(f"/sys/class/hidraw/{dxi}/device")
        dev_dir = base.resolve().parent.parent
        bcdDevice = int((dev_dir / "bcdDevice").read_text().strip(), 16)
        return bcdDevice


    def set_feature(self, rid, cmd, payload=b""):
        buf = bytearray(64)
        buf[0] = rid
        buf[1] = cmd
        buf[2] = len(payload)
        buf[3:3 + len(payload)] = bytes(payload)
        fcntl.ioctl(self.fd, _IOC_SF(64), buf, True)

    def get_feature(self, rid, max_len=64):
        buf = bytearray(max_len)
        buf[0] = rid
        try: 
            fcntl.ioctl(self.fd, _IOC_GF(max_len), buf, True)
        except: 
            # wireless connections sometimes take two attempts ...
            time.sleep(0.05)
            fcntl.ioctl(self.fd, _IOC_GF(max_len), buf, True)
        return bytes(buf)

    def read_attribute_values(self, rid, cmd):
        self.set_feature(rid, cmd)
        data = self.get_feature(rid, 256)   # Feature report with attributes can be longer than 64.

        data = data[3:]
        attribute_count = len(data) // 5
        padding_len = len(data) - (attribute_count * 5)

        fstr = '<' + 'BL' * attribute_count + 'B' * padding_len
        unpacked = struct.unpack(fstr, data)

        attribute_list = {}
        for i in range(attribute_count):
            tag = unpacked[2*i]
            val = unpacked[2*i + 1]
            attribute_list[tag] = val

        return attribute_list

    def read_firmware_version(self, force_esb = False):
        # When sent to a Puck, force_esb=False will return the puck's firmware, 
        # while force_esb=True will return the connected controller's firmware.

        if self.pid in CTRL_PIDS or force_esb:
            attrs = self.read_attribute_values(1, 0x83)

        elif self.pid in PUCK_PIDS:
            if self.get_bcd_version() == 2:
                attrs = self.read_attribute_values(2, 0x83)
            else: 
                # I wonder which official 
                # Valve device uses a BCD other than 2. Maybe a prototype?
                attrs = self.read_attribute_values(1, 0xa6)

        return attrs[4]

    def read_serial(self, force_esb = False, index = 1):
        # When sent to a Puck, force_esb=False will return the puck's serial, 
        # while force_esb=True will return the connected controller's serial.

        # Index:
        # 0: MXA123123123A ??
        # 1: FXB9912345678 (serial number on sticker)
        # 2: NA
        # 3: fec01234c8af git firmware commit hash
        # 4: MXA123123123A

        r = None

        if self.pid in CTRL_PIDS or force_esb:
            self.set_feature(1, 0xae, bytes([index]))
            r = self.get_feature(1)
              
        elif self.pid in PUCK_PIDS:
            if self.get_bcd_version() == 2:
                # 2 = Valve Puck or Machine, 211-213 OpenPuck? Unclear why.
                self.set_feature(2, 0xae, bytes([index]))
                r = self.get_feature(2)
            else: 
                # I wonder which official Valve device uses 
                # a BCD other than 2. Maybe a prototype?
                self.set_feature(1, 0xa4, bytes([index]))
                r = self.get_feature(1)

        if r is None: 
            return None
        
        return r[4:].split(b"\x00")[0].decode("latin1", "replace")
            
       

def nodesort(nodeobj):
    """Sorts the node path properly (numerically)."""

    # TODO: This doesn't seem to be 100% reliable, if you plug in two Pucks at the exact same time
    # the interface order might get messed up.

    if isinstance(nodeobj, HidDev):
        return nodesort(nodeobj.node)

    if isinstance(nodeobj, str):
        return int(nodeobj.replace("/dev/hidraw", ""))


def enumerate_devices():
    """Return dict role -> list[HidDev]: {'puck':[...], 'ctrl':[...]}."""
    out = {"puck": [], "ctrl": []}

    last_serial = None
    current_ctrl_iface = None
    current_esb_ifaces = []

    for node in sorted(glob.glob("/dev/hidraw*"), key=nodesort):
        vid, pid = _sysfs_ids(node)
        if vid != VALVE_VID:
            continue

        # Check if this hidraw object is actually a puck slot, or if it's a OpenPuck Wake mouse or similar
        dxi = node.replace('/dev/', '')
        base = Path(f"/sys/class/hidraw/{dxi}/device")
        iface_dir = base.resolve().parent

        bInterfaceClass = int((iface_dir / "bInterfaceClass").read_text().strip(), 16)
        bInterfaceSubClass = int((iface_dir / "bInterfaceSubClass").read_text().strip(), 16)
        bInterfaceProtocol = int((iface_dir / "bInterfaceProtocol").read_text().strip(), 16)

        # A real puck slot has these values:
        if (bInterfaceClass != 3 or bInterfaceSubClass != 0 or bInterfaceProtocol != 0):
            # If these don't match, this is not a Puck slot, but some other random HID interface.
            continue

        # If we're here, this is a Puck. Let's detect and display its type.
        # OpenPuck can be detected by the OpenPuck mouse wake interface. 

        devnode = iface_dir.parent
        device_vendor = "Valve Puck"

        if pid == 0x1305:
            device_vendor = "Valve Steam Machine"
        else:
            for subnode in glob.glob(str(devnode) + "/" + devnode.name + ":*/interface"):
                txt = Path(subnode).read_text().strip()
                if "OpenPuck" in txt:
                    device_vendor = "OpenPuck"

        try:
            dev = HidDev(node, vid, pid, device_vendor)
        except PermissionError:
            print("permission denied on %s (run with sudo or install the udev rule)" % node, file=sys.stderr)
            continue
        except Exception:
            continue

        if pid in CTRL_PIDS:
            out["ctrl"].append(dev)
            continue

        serial = dev.read_serial()

        if pid in PUCK_PIDS and last_serial is not None and last_serial != serial:
            # OpenCode has no control interface.          
            current_puck = {
                "control": current_ctrl_iface,
                "esb": current_esb_ifaces
            }

            out["puck"].append(current_puck)
            current_esb_ifaces = []
            current_ctrl_iface = None

        last_serial = serial


        if pid in PUCK_PIDS and dev.usage_page in (0x01, None):
            current_esb_ifaces.append(dev)
        elif pid in PUCK_PIDS:
            current_ctrl_iface = dev
        elif pid in CTRL_PIDS:
            out["ctrl"].append(dev)
        else:
            dev.close()


    if len(current_esb_ifaces) > 0:
        # Add puck interfaces from last loop
        # OpenCode has no control interface.          

        current_puck = {
            "control": current_ctrl_iface,
            "esb": current_esb_ifaces
        }

        out["puck"].append(current_puck)

    return out


def _ser16(serial):
    b = serial.encode("latin1", "replace")[:16]
    return b + b"\x00" * (16 - len(b))


def read_slots(puck_list):
    """Read all bond slots from all attached pucks (each puck control interface == one slot)."""
    pucks = []

    for puck in puck_list:

        esb_devices = puck['esb']

        slots = []
        idx = 0

        for esb in esb_devices:
            try:

                esb.set_feature(2, 0xA3)
                r = esb.get_feature(2)  # [02 A3 18 <8 uuid><16 serial>]
                rec = r[3:3 + 24]
                used = any(rec[0:8])
                serial = rec[8:24].split(b"\x00")[0].decode("latin1", "replace") if used else ""
                key = rec[:8].hex()
    
                # Check if a controller is currently connected to this slot:
                esb.set_feature(2, 0xB4)
                r = esb.get_feature(2)
                active = (r[0] == 0x02 and r[1] == 0xb4 and r[3] == 2)
    
                slots.append({"idx": idx, "dev": esb, "used": used, "active": active, "serial": serial, "rec": rec})
            except Exception as ex:
                slots.append({"idx": idx, "dev": esb, "used": False, "active": False, "serial": "", "rec": b"", "err": str(ex)})

            idx = idx + 1
            
        if esb is not None:
            # This is a new puck, append the old puck and all its slots to the list.
            pucks.append({"serial": esb.read_serial(), 
                        "firmware_ts": esb.read_firmware_version(),
                        "firmware_commit": esb.read_serial(False, 3),
                        "slots": slots, 
                        "info_str": esb.info_str})
        
    return pucks


def get_puck_slot_for_writing(puck_serial=None, slot=None):
    devs = enumerate_devices()
    pucks = devs["puck"]

    if not pucks:
        raise SystemExit("no puck (28DE:1304/1305) control interface found")

    puck_objects = read_slots(pucks)
    if puck_serial is None and len(puck_objects) > 1:
        raise SystemExit("Multiple Pucks connected, you need to specify its serial number")
    
    puck_object = None

    if puck_serial is not None:
        for puck in puck_objects:
            if puck['serial'] == puck_serial:
                puck_object = puck
                break
        
        if puck_object is None: 
            raise SystemExit("Can't find a puck with that serial number!")
        
    else: 
        puck_object = puck_objects[0]


    slots = puck_object['slots']

    if slot is None:
        slot_object = next((s for s in slots if not s["used"]), None)
    else: 
        slot_object = next((s for s in slots if s["idx"] == slot), None)

    if slot_object is None:
        raise SystemExit("This Puck has all slots occupied!")

    
    return (puck_object['serial'], slot_object)
    


def pair(puck_serial=None, puck_slot=None, controller_serial=None, controller_slot=0, puck_name=None, controller_name=None, pkey=None):
    devs = enumerate_devices()
    puck_devs = devs["puck"]
    pucks = []

    for puck in puck_devs:
        pucks.append(puck["esb"])

    ctrls = sorted(devs["ctrl"], key=nodesort)
    if not pucks:
        raise SystemExit("no puck (28DE:1304/1305) control interface found")
    
    ctrl = None
    if controller_serial is not None:
        # Check controllers connected through USB
        for c in ctrls:
            if c.read_serial() == controller_serial:
                print(f"Found controller {controller_serial} connected over USB")
                ctrl = c

        if ctrl is None: 
            # Check controllers connected wirelessly
            for puck in pucks:
                for slot in puck:
                    try: 
                        if slot.read_serial(True) == controller_serial:
                            print(f"Found controller {controller_serial} connected wirelessly")
                            ctrl = puck
                    except BrokenPipeError:
                        pass
    else: 
        # Controller serial is none. Assume the user wants to use the 1st wired controller. 
        if not ctrls:
            raise SystemExit("no controller (28DE:1302) control interface found - please plug in either a Steam Controller or the ReversePuck controller")
           
        ctrl = ctrls[0]

    if ctrl is None:
        raise SystemExit(f"Controller with serial {controller_serial} not found.")


    if controller_name is None: 
        ctrl_serial_str = ctrl.read_serial(True)
    else: 
        ctrl_serial_str = controller_name[:15]

    puck_serial, slot_obj = get_puck_slot_for_writing(puck_serial, puck_slot)  

    if puck_name is None: 
        puck_serial_str = puck_serial
    else: 
        puck_serial_str = puck_name[:15]

    if pkey is None: 
        r = os.urandom(8)
    else: 
        r = pkey

    key = b"esb/bond\x00" if controller_slot == 0 else b"esb/bond_2\x00"

    print("Pairing key: %s" % (r.hex()))

    # Puck name gets written to controller (defaults to puck serial).
    # Controller name gets written to Puck

    print("Puck: %s slot %d controller %s" % (puck_serial, slot_obj['idx'], ctrl_serial_str))
    print("Controller: %s slot %d puck %s" % (ctrl.read_serial(True), controller_slot, puck_serial_str))

    # puck slot: 0xA2 [r1 r2][ctrl serial]
    slot_obj['dev'].set_feature(2, 0xA2, r + _ser16(ctrl_serial_str))
    # controller esb/bond: 0xEE [key\0][r1 r2][puck serial] + 0xEF [key\0]
    ctrl.set_feature(1, 0xEE, key + r + _ser16(puck_serial_str))
    ctrl.set_feature(1, 0xEF, key)
    time.sleep(0.1)
    # reboot controller into wireless mode (magic 0xA427AF52)
    ctrl.set_feature(1, 0x95, bytes([0x52, 0xAF, 0x27, 0xA4]))
    print("paired. controller rebooting into wireless — move the puck to your host.")

def write_puck_slot(puck_serial=None, puck_slot=None, ctrl_name=None, pkey=None):

    puck_serial, slot_obj = get_puck_slot_for_writing(puck_serial, puck_slot)  

    if ctrl_name is None and pkey is None: 
        print("wiping puck %s slot %d" % (puck_serial, slot_obj['idx']))
        slot_obj['dev'].set_feature(2, 0xA2, b"\x00" * 24)
    else:

        if ctrl_name is None or len(ctrl_name) > 15: 
            raise SystemExit("Invalid controller name.")

        if pkey is None: 
            raise SystemExit("No key provided!")
        
        print("writing to puck %s slot %d: controller %s key=%s" %
          (puck_serial, slot_obj['idx'], ctrl_name, pkey.hex()))
        # puck slot: 0xA2 [r1 r2][ctrl serial]

        slot_obj['dev'].set_feature(2, 0xA2, pkey + _ser16(ctrl_name))

    print("done")

def write_controller_slot(controller_serial=None, slot=0, puck_serial=None, pkey=None):

    devs = enumerate_devices()
    puck_devs = devs["puck"]
    pucks = []

    for puck in puck_devs:
        pucks.append(puck["esb"])

    ctrls = sorted(devs["ctrl"], key=nodesort) 
    ctrl = None
    if controller_serial is not None:
        # Check controllers connected through USB
        for c in ctrls:
            if c.read_serial() == controller_serial:
                print(f"Found controller {controller_serial} connected over USB")
                ctrl = c

        if ctrl is None: 
            # Check controllers connected wirelessly
            for puck in pucks:
                for slot in puck:
                    print(slot)
                    try: 
                        if slot.read_serial(True) == controller_serial:
                            print(f"Found controller {controller_serial} connected wirelessly")
                            ctrl = puck
                    except BrokenPipeError:
                        pass
    else: 
        # Controller serial is none. Assume the user wants to use the 1st wired controller. 
        if not ctrls:
            raise SystemExit("no controller (28DE:1302) control interface found - please plug in a Steam Controller")
           
        ctrl = ctrls[0]

    if ctrl is None:
        raise SystemExit(f"Controller with serial {controller_serial} not found.")


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

def get_single_controller_pairings(controller_dev):

    slots = []
    i = 0
    for bond_str in [b"esb/bond\x00", b"esb/bond_2\x00"]:
        controller_dev.set_feature(1, 0xED, bond_str)
        bond = controller_dev.get_feature(1)

        if bond[0] == 0x01 and bond[1] == 0xed and bond[2] > 0:
            bond_len = bond[2]
            bond_key = bond[3:3+8]
            bond_name = bond[11:11+(bond_len-8)]
            slots.append({"idx": i, "used": True, "key": bond_key.hex(), "serial": bond_name.decode("latin1", "replace").split('\x00')[0]})
        else: 
            slots.append({"idx": i, "used": False})

        i = i + 1

    return {"serial": controller_dev.read_serial(True), 
            "firmware_ts": controller_dev.read_firmware_version(True), 
            "firmware_commit": controller_dev.read_serial(True, 3), 
            "slots": slots}


def cmd_reboot(device_serial: str, reboot_type: int):
    """
        type:
        These types don't correspond to anything in the protocol, these are just flags used for this function.
        
        1: reboot (controller only)
        2: reboot to bootloader (puck or controller)
        3: reboot to wireless connection (controller only)
        4: power off (controller only)
    
    """

    devs = enumerate_devices()

    puck_devs = sorted(devs["puck"], key=nodesort)

    ctrls = sorted(devs["ctrl"], key=nodesort)


    # Check if we want to reboot a Puck
    for p in puck_devs: 
        pc = p['control']
        if pc.read_serial() == device_serial:
            if reboot_type == 2:
                print(f"Found puck with serial {device_serial}, rebooting to bootloader...")
                if pc.get_bcd_version() == 2:
                    pc.set_feature(2, 0x90)
                else: 
                    pc.set_feature(1, 0x90)
                return True
            else:
                print(f"Found puck with serial {device_serial}, but pucks can only reboot to bootloader. Skipping.")
                print(f"If that's what you want, run again with --bootloader.")

    # Check if we want to reboot a controller that's wireless.
    found_puck_dev = None
    ps = read_slots(puck_devs)
    for puck in ps:
        slots = puck['slots']

        for s in slots: 
            if s["active"]:
                if s['dev'].read_serial(True) == device_serial:
                    found_puck_dev = s['dev']
                    break

    # Check if we want to reboot a controller that's wired.
    if found_puck_dev is None:
        for c in ctrls: 
            if c.read_serial() == device_serial:
                found_puck_dev = c
                break

    if found_puck_dev is not None:
        if reboot_type == 1: 
            print(f"Found controller with serial {device_serial}, rebooting...")
            found_puck_dev.set_feature(1, 0x95)
        elif reboot_type == 2:
            print(f"Found controller with serial {device_serial}, rebooting to bootloader...")
            found_puck_dev.set_feature(1, 0x90)
        elif reboot_type == 3:
            print(f"Found controller with serial {device_serial}, rebooting to wireless...")
            found_puck_dev.set_feature(1, 0x95, bytes([0x52, 0xAF, 0x27, 0xA4]))
        elif reboot_type == 4:
            print(f"Found controller with serial {device_serial}, powering off...")
            found_puck_dev.set_feature(1, 0x9F, b'off!')
        return True


def cmd_list():
    devs = enumerate_devices()
    pucks = devs["puck"]
    ctrls = devs["ctrl"]

    ctrls_esb = []

    if pucks:
        ps = read_slots(pucks)
        for puck in ps:
            slots = puck['slots']

            firmware_str = hex(puck['firmware_ts']).replace('0x', '').upper() + " (" + puck['firmware_commit']  + ")"

            print(f"Puck: {puck['serial']} ({puck['info_str']}), firmware {firmware_str}")

            for s in slots: 
                suffix = ""
                if s["active"]:
                    conn_serial = s['dev'].read_serial(True)
                    suffix = f", connected to controller " + conn_serial
                    ctrls_esb.append(s['dev'])

                if s["used"]:
                    print("  slot %d: %s (key %s)%s" % (s["idx"], s["serial"], s["rec"][:8].hex(), suffix))
                else: 
                    print("  slot %d: Empty" % (s["idx"]))

            print()

    else:
        print("No Puck connected.")


    if ctrls or ctrls_esb:
        ctype = "wired"
        for c in [*ctrls, None, *ctrls_esb]:
            if c is None: 
                ctype = "wireless"
                continue

            cdata = get_single_controller_pairings(c)

            firmware_str = hex(cdata['firmware_ts']).replace('0x', '').upper() + " (" + cdata['firmware_commit']  + ")"
        
            print(f"Controller ({ctype}): {cdata['serial']}, firmware {firmware_str}")

            for s in cdata['slots']:
                if s['used']:
                    print("  slot %d: %s (key %s)" % (s["idx"], s["serial"], s["key"]))
                else:
                    print("  slot %d: Empty" % (s["idx"]))

            print()

    else:
        print("No controller connected.")




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
    #def pair(puck_serial=None, puck_slot=None, controller_serial=None, controller_slot=0, 
    #           puck_name=None, controller_name=None, pkey=None)
    p.add_argument("--puck-serial", type=str, default=None)
    p.add_argument("--puck-slot", type=int, default=None)
    p.add_argument("--puck-name", type=str, default=None)
    p.add_argument("--controller-serial", type=str, default=None)
    p.add_argument("--controller-slot", type=int, default=0)
    p.add_argument("--controller-name", type=str, default=None)
    p.add_argument("--key", type=steam_key_check, default=None)


    s = sub.add_parser("write-puck")
    #def write_puck_slot(puck_serial=None, puck_slot=None, ctrl_name=None, pkey=None):
    s.add_argument("--puck-serial", type=str, default=None)
    s.add_argument("--puck-slot", type=int, default=None)
    s.add_argument("--controller-name", type=str, default=None)
    s.add_argument("--key", type=steam_key_check, default=None)

    c = sub.add_parser("write-controller")
    #def write_controller_slot(controller_serial=None, slot=0, puck_serial=None, pkey=None):
    c.add_argument("--controller-serial", type=str, default=None)
    c.add_argument("--controller-slot", type=int, default=None)
    c.add_argument("--puck-name", type=str, default=None)
    c.add_argument("--key", type=steam_key_check, default=None)


    rb = sub.add_parser("reboot")
    #def cmd_reboot(device_serial: str, reboot_type: int):
    rb.add_argument("--serial", type=str, required=True)

    reboot_group = rb.add_mutually_exclusive_group()        # no parameters: normal reboot, type 1
    reboot_group.add_argument("--bootloader", action='store_true') # type 2
    reboot_group.add_argument("--wireless", action='store_true') # type 3
    reboot_group.add_argument("--poweroff", action='store_true') # type 4



    
    args = ap.parse_args()
    if args.cmd == "list":
        cmd_list()
    elif args.cmd == "pair":
        pair(args.puck_serial, args.puck_slot, args.controller_serial, 
                args.controller_slot, args.puck_name, args.controller_name, args.key)
    elif args.cmd == "write-puck":
        write_puck_slot(args.puck_serial, args.puck_slot, args.controller_name, args.key)
    elif args.cmd == "write-controller":
        write_controller_slot(args.controller_slot, args.puck_name, args.key)
    elif args.cmd == "reboot":
        reboot_type = 1
        if args.bootloader: 
            reboot_type = 2
        if args.wireless: 
            reboot_type = 3
        if args.poweroff: 
            reboot_type = 4

        cmd_reboot(args.serial, reboot_type)



if __name__ == "__main__":
    main()
