#!/usr/bin/env python3
"""openctrl.py -- Steam Deck forwarder for the OpenController emulated Steam Controller 2.

Runs on the Steam Deck (Desktop Mode). Opens the nRF (Valve 28DE:1302) CDC port, reads its STATUS
stream (which bonded pucks are live), and shows a fullscreen touchscreen UI of tappable puck tiles.
Tap an *available* puck -> the Deck's controls are grabbed (detached) and streamed to the nRF, which
relays them over RF so the puck presents a Steam Controller 2 to its host. Tap again, or lose the RF
link, and control returns to the Deck.

    python3 openctrl.py                 # fullscreen touchscreen UI
    python3 openctrl.py --debug         # headless: print status, no UI (for bring-up)
    python3 openctrl.py --port /dev/ttyACM0   # override CDC port autodetect
    python3 openctrl.py --evdev         # use the evdev fallback input source (lower fidelity)

The dongle (nRF CDC) is hot-pluggable: launch with or without it, it appears/disappears live.
Deps: pyserial + pygame (UI) + pyusb (input/USB detach). See README.md / run.sh.
"""
import argparse
import os
import sys
import time

import frame
import input_source as isrc

VID = 0x28DE
PID = 0x1302  # the OpenController emulated controller
INPUT_HZ = 250.0


def find_port():
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        if (p.vid, p.pid) == (VID, PID):
            return p.device
    # fall back: any Valve CDC
    for p in list_ports.comports():
        if p.vid == VID:
            return p.device
    return None


class App:
    def __init__(self, args):
        self.args = args
        import serial  # imported here so --help works without pyserial
        self._serial = serial

        # The dongle (nRF CDC) is HOT-PLUGGABLE: we don't require it at launch and we re-open it
        # whenever it (re)appears, and tear down cleanly when it goes away.
        self.ser = None
        self.port_name = None
        self.reader = frame.Reader()
        self._last_open_try = 0.0

        self.input = None
        self.forwarding = False
        self.fwd_slot = None
        self.last_input = 0.0

        # model surfaced to the UI
        self.status = {"link_up": False, "link_slot": None, "sess_ch": 0,
                       "bonds": [], "forwarding": False}
        self.note = "waiting for OpenController dongle (28DE:1302)…"
        self.log = []  # firmware '#' diagnostic lines (the in-app serial monitor)

        # file log: everything (firmware '#' lines + app events) with timestamps, for sharing.
        self.logf = None
        try:
            self.logf = open(args.logfile, "a", buffering=1)  # line-buffered
            self._flog("==== openctrl start ====")
        except Exception as ex:
            print("logfile open failed (%s); continuing without it" % ex)

    def _flog(self, line):
        if self.logf:
            try:
                self.logf.write("%.3f %s\n" % (time.monotonic(), line))
            except Exception:
                pass

    @property
    def dongle_connected(self):
        return self.ser is not None

    # ---- serial I/O (hot-plug aware) ----
    def _open_serial(self):
        """Try to (re)open the dongle CDC port. Throttled to ~2 Hz. Sets self.ser on success."""
        if self.ser is not None:
            return
        now = time.monotonic()
        if now - self._last_open_try < 0.5:
            return
        self._last_open_try = now
        port = self.args.port or find_port()
        if not port:
            return
        try:
            self.ser = self._serial.Serial(port, 115200, timeout=0)
            self.port_name = port
            self.reader = frame.Reader()  # fresh framer for the new connection
            # forget any stale link state from a previous dongle session
            self.status = {"link_up": False, "link_slot": None, "sess_ch": 0,
                           "bonds": [], "forwarding": False}
            self.note = "dongle connected: %s" % port
            self._flog("dongle connected: %s" % port)
        except Exception:
            self.ser = None  # e.g. permission / busy; will retry

    def _drop_serial(self, why):
        """Tear down on disconnect: stop forwarding, close the port, reset the model."""
        if self.forwarding:
            self.stop_forwarding()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.port_name = None
        self.status = {"link_up": False, "link_slot": None, "sess_ch": 0,
                       "bonds": [], "forwarding": False}
        self.note = "dongle disconnected (%s) — waiting…" % why
        self._flog("dongle disconnected: %s" % why)

    def pump_serial(self):
        if self.ser is None:
            self._open_serial()
            return
        # A yanked CDC often makes pyserial return b'' silently rather than raise, so the port node
        # disappearing is the reliable disconnect signal. Check it (throttled).
        now = time.monotonic()
        if now - self._last_open_try >= 0.5:
            self._last_open_try = now
            if self.port_name and not os.path.exists(self.port_name):
                self._drop_serial("unplugged")
                return
        try:
            data = self.ser.read(512)
        except Exception as ex:
            # device yanked / I/O error -> clean teardown, then we'll re-detect it
            self._drop_serial(str(ex) or "read error")
            return
        last_haptic = None
        for typ, payload in self.reader.feed(data):
            if typ == frame.T_STATUS:
                st = frame.parse_status(payload)
                if st:
                    self.status = st
            elif typ == frame.T_HAPTIC:
                # coalesce a burst to the newest state -- replaying every frame is what lagged the rumble
                last_haptic = frame.parse_haptic(payload)
            elif typ == frame.T_TEXT:
                line = payload.decode("latin1", "replace").rstrip()
                if line:
                    self.note = line
                    self.log.append(line)
                    self.log = self.log[-300:]
                    self._flog(line)
                    if self.args.debug:
                        print(line)  # stream the firmware log like a serial monitor
        # play only the newest haptic of this batch, and only while we're actually forwarding
        if last_haptic and self.forwarding and self.input:
            try:
                self.input.rumble(last_haptic["intensity"], last_haptic["left"],
                                  last_haptic["right"], last_haptic["lgain"],
                                  last_haptic["rgain"])
            except Exception:
                pass

    def send(self, b):
        if self.ser is None:
            return
        try:
            self.ser.write(b)
        except Exception:
            self._drop_serial("write error")

    # ---- forwarding control ----
    def _ensure_input(self):
        if self.input is None:
            # libusb DeckSource by default (detaches the pad from Steam + decodes the raw report
            # correctly); --evdev forces the lower-fidelity fallback.
            self.input = isrc.open_source(prefer_evdev=self.args.evdev,
                                          path=self.args.input)
        return self.input

    def start_forwarding(self, slot):
        try:
            self._ensure_input().grab()
        except Exception as ex:
            self.note = "grab failed: %s" % ex
            return
        self.forwarding = True
        self.fwd_slot = slot
        self.send(frame.build_set_forwarding(True))
        self.note = "forwarding -> slot %s (%s)" % (slot, self.input.name)
        self._flog("APP forwarding START slot=%s input=%s" % (slot, self.input.name))

    def stop_forwarding(self):
        self.forwarding = False
        self.fwd_slot = None
        self.send(frame.build_set_forwarding(False))
        if self.input:
            self.input.ungrab()
        self.note = "control returned to Deck"
        self._flog("APP forwarding STOP")

    def toggle(self, slot):
        """Tap handler: a tappable tile is one whose puck is live (alive)."""
        if self.forwarding:
            self.stop_forwarding()
        elif self.dongle_connected:
            self.start_forwarding(slot)

    def remove_bond(self, slot):
        """GUI "remove": un-bond a puck slot on the firmware. Refused while forwarding to it."""
        if self.forwarding and self.fwd_slot == slot:
            return
        self.send(frame.build_clear_bond(slot))
        # drop it from the local model immediately so the tile disappears without waiting for STATUS
        self.status["bonds"] = [b for b in self.status.get("bonds", []) if b["slot"] != slot]
        self._flog("APP remove bond slot=%s" % slot)

    def pump_input(self):
        if not self.forwarding or not self.input:
            return
        st = self.input.pump()
        now = time.monotonic()
        if now - self.last_input >= 1.0 / INPUT_HZ:
            self.last_input = now
            self.send(frame.build_input(st))

    def auto_release_on_drop(self):
        """If the RF link drops while forwarding, hand control back to the Deck."""
        if self.forwarding and not self.status.get("link_up"):
            # only release if the slot we're forwarding to is no longer alive
            slot = self.fwd_slot
            bonds = self.status.get("bonds", [])
            alive = any(b["slot"] == slot and b["alive"] for b in bonds)
            if not alive:
                self.stop_forwarding()

    # ---- loops ----
    def run_debug(self):
        last = 0
        while True:
            self.pump_serial()
            self.pump_input()
            self.auto_release_on_drop()
            if time.monotonic() - last > 0.5:
                last = time.monotonic()
                s = self.status
                tiles = ", ".join(
                    "[%d %s%s]" % (b["slot"], b["serial"] or "?", " LIVE" if b["alive"] else "")
                    for b in s["bonds"]) or "(no bonds)"
                print("dongle=%s link=%s fwd=%s ch=%s %s | %s" %
                      ("yes" if self.dongle_connected else "NO", s["link_up"],
                       self.forwarding, s["sess_ch"], tiles, self.note))
            time.sleep(0.002)

    def run_ui(self):
        import ui
        ui.run(self)


def main():
    ap = argparse.ArgumentParser(description="Steam Deck forwarder for OpenController")
    ap.add_argument("--port", help="CDC port (default: autodetect Valve 28DE:1302)")
    ap.add_argument("--input", help="force an evdev node, e.g. /dev/input/event12 (only with --evdev)")
    ap.add_argument("--evdev", action="store_true",
                    help="use the evdev fallback instead of the libusb DeckSource (lower fidelity, "
                    "no USB-level detach)")
    ap.add_argument("--debug", action="store_true", help="headless status print, no UI")
    ap.add_argument("--logfile", default="openctrl.log",
                    help="append firmware log + app events here (default: openctrl.log in this dir)")
    args = ap.parse_args()
    app = App(args)
    try:
        if args.debug:
            app.run_debug()
        else:
            app.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        if app.forwarding:
            app.stop_forwarding()


if __name__ == "__main__":
    main()
