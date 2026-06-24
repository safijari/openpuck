#!/bin/sh
# gen_uf2.sh -- convert a compiled .hex into a .uf2 for drag-and-drop flashing.
#
# The Adafruit nRF52 core ships uf2conv.py but leaves the UF2 objcopy recipe
# COMMENTED OUT in platform.txt (recipe.objcopy.uf2.pattern), so `arduino-cli
# compile` only ever emits .hex/.zip and never a .uf2. This regenerates the
# .uf2 from the .hex after the compile step.
#
# Usage: ./gen_uf2.sh <input.hex> <output.uf2> [family]
set -e

HEX="$1"
OUT="$2"
# nRF52840 UF2 family id (boards.txt: feather52840.build.uf2_family=0xADA52840).
FAMILY="${3:-0xADA52840}"

if [ -z "$HEX" ] || [ -z "$OUT" ]; then
	echo "usage: $0 <input.hex> <output.uf2> [family]" >&2
	exit 2
fi
if [ ! -f "$HEX" ]; then
	echo "gen_uf2.sh: input hex not found: $HEX" >&2
	exit 1
fi

# Locate uf2conv.py inside the installed Adafruit nRF52 core. Honor a custom
# Arduino data dir (config get), falling back to the per-OS defaults.
DATA_DIR="$(arduino-cli config get directories.data 2>/dev/null || true)"
if [ -z "$DATA_DIR" ]; then
	if [ -d "$HOME/Library/Arduino15" ]; then
		DATA_DIR="$HOME/Library/Arduino15"
	else
		DATA_DIR="$HOME/.arduino15"
	fi
fi

UF2CONV="$(find "$DATA_DIR/packages/adafruit/hardware/nrf52" -name uf2conv.py 2>/dev/null | sort | tail -n1)"
if [ -z "$UF2CONV" ]; then
	echo "gen_uf2.sh: could not find uf2conv.py under $DATA_DIR" >&2
	exit 1
fi

# Prefer python3; fall back to a bare python (the CI runner's setup-python
# provides `python`, while macOS only ships `python3`). uf2conv.py is py3-safe.
PYTHON="$(command -v python3 || command -v python || true)"
if [ -z "$PYTHON" ]; then
	echo "gen_uf2.sh: no python interpreter found" >&2
	exit 1
fi

"$PYTHON" "$UF2CONV" -f "$FAMILY" -c -o "$OUT" "$HEX"
echo "gen_uf2.sh: wrote $OUT (family $FAMILY)"
