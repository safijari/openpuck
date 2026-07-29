// usb_mount.h -- dynamic USB mounting of ACTIVELY-CONNECTED controllers (emulated modes only).
//
// Problem: a puck can be bonded to several controllers but only some are powered on / linked at a time. The
// emulated modes used to mount one USB HID gamepad per BONDED slot at boot, so the host saw phantom pads for
// controllers that were off. This module makes the USB device present only the CONNECTED controllers, and
// re-enumerate (NO MCU reboot -- RF links stay up) whenever that set changes.
//
// Model: a fixed pool of per-mode USB-slot objects (HID/XInput interfaces) is created ONCE at boot (locking
// their TinyUSB instance indices). At any moment the k connected controllers are presented as a DENSE PREFIX
// usbSlot 0..k-1 of that pool; usbSlot u is fed from bond slot g_usbToBond[u]. USB can't hold an interface
// "gap", so the prefix is compacted -> player identity follows CONNECTION ORDER (a mid-set disconnect shifts
// the rest down; a reconnect appends). Re-enumeration is debounced so an RF blip doesn't reshuffle.
#pragma once
#include <stdint.h>
#include "bonds.h" // NSLOT

// Active mount map. usbSlot (0..g_usbMountCount-1) -> bond slot; and the inverse for output-report routing.
extern uint8_t g_usbMountCount; // # of currently-mounted (connected) controllers
extern int8_t g_usbToBond[NSLOT]; // usbSlot -> bondSlot (-1 = unused)
extern int8_t g_bondToUsb[NSLOT]; // bondSlot -> usbSlot (-1 = not mounted)

// Enable the dynamic watcher for this boot (emulated modes call this; puck/lizard leave it off so their begin()
// keeps the legacy behavior). maxSlots caps the mounted count to the mode's HID-instance budget (CFG_TUD_HID
// minus the wake mouse, etc.). When disabled, usbMountTask() is a no-op.
void usbMountEnable(bool on, uint8_t maxSlots);

// Build g_usbToBond/g_bondToUsb + g_usbMountCount from the CURRENT connected set (bond-slot order). Called by
// the boot enumerate and on every accepted change.
void usbMountRebuildMap();

// Per-loop watcher: detect a debounced change in the connected set and trigger usbReenumerate(k). No-op unless
// dynamic mounting is enabled.
void usbMountTask();

// Provided by OpenPuck.ino: tear down + rebuild the USB config descriptor presenting `k` slot interfaces
// (fixed interfaces -- WebUSB / wake mouse -- replayed too), then re-attach. NO MCU reboot. Call with the map
// already rebuilt for k connected controllers.
void usbReenumerate(uint8_t k);

// Persist `mode` and reboot into it, cleanly detaching USB first so the host tears down the outgoing
// personality (releasing any held input) before the new one enumerates -- otherwise the ~ms reset-disconnect
// can be too brief for the host to notice, leaving a ghost device with stuck input until a physical replug.
// Does not return. Call for every user-facing mode switch (chord / panel / console).
void modeSwitchReboot(uint8_t mode);
