#!/usr/bin/env bash
# Per-boot device-permission grant for SteamOS (read-only rootfs -> no persistent udev rule).
# Run ONCE PER BOOT from Desktop Mode:   sudo ./setup.sh
#
# Opens, for this boot only (nothing on the read-only FS changes -- re-run after a reboot):
#   * /dev/ttyACM*                      -- the nRF (OpenController) CDC link
#   * the Deck controller's usbfs node  -- so the app's libusb DeckSource can detach + read it
#                                          (28DE:1205); this is what lets it take the pad from Steam
# /dev/input/event* is already granted to the active session (logind ACL) for the --evdev fallback.
set -e
shopt -s nullglob

for f in /dev/ttyACM*; do chmod a+rw "$f" && echo "opened CDC $f"; done

# Find the Steam Deck controller (Valve 28DE:1205) usbfs node and open it for libusb.
opened=0
for dd in /sys/bus/usb/devices/*; do
	[ -r "$dd/idVendor" ] || continue
	if [ "$(cat "$dd/idVendor" 2>/dev/null)" = "28de" ] &&
	   [ "$(cat "$dd/idProduct" 2>/dev/null)" = "1205" ]; then
		busnum=$(cat "$dd/busnum"); devnum=$(cat "$dd/devnum")
		node=$(printf "/dev/bus/usb/%03d/%03d" "$busnum" "$devnum")
		chmod a+rw "$node" && echo "opened controller usbfs $node ($dd)" && opened=1
	fi
done
[ "$opened" = 1 ] || echo "WARNING: Deck controller 28DE:1205 not found on USB (is it the built-in pad?)"

echo "done (this boot only)."
