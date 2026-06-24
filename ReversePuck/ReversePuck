#!/usr/bin/env bash
# OpenController Steam Deck forwarder launcher (uv-based; nothing installed system-wide).
#
# Add as a NON-STEAM GAME:
#   Target:      /home/deck/…/deck/run.sh
#   Start In:    /home/deck/…/deck/         (the directory this script is in)
# Then, in that shortcut's gear -> Properties -> Controller, set "Disable Steam Input" so Steam doesn't
# fight us for the built-in pad while we grab it.
#
# Dependencies are pulled by uv on first run (pyserial + pygame + pyusb -- prebuilt wheels; pyusb uses
# the system libusb-1.0.so). uv also provisions its own Python, so nothing touches the read-only rootfs.
#
# USB-level detach (DeckSource) needs access to the controller's usbfs node, and the CDC link needs
# /dev/ttyACM*. Run `sudo ./setup.sh` once per boot to open both (see README). Without it, fall back to
# `./run.sh --evdev` (no USB detach; Steam/QAM won't forward, trackpads are lower fidelity).
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

UV="$(command -v uv || true)"
[ -n "$UV" ] || UV="$HOME/.local/bin/uv"
[ -x "$UV" ] || { echo "uv not found. Install it: https://docs.astral.sh/uv/  (curl -LsSf https://astral.sh/uv/install.sh | sh)"; exit 1; }

exec "$UV" run --with pyserial --with pygame --with pyusb python openctrl.py "$@"
