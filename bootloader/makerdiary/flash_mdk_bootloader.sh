#!/usr/bin/env bash
set -euo pipefail

usage()
{
	cat <<'USAGE'
Usage: flash_mdk_bootloader.sh PORT [PACKAGE] [BAUD]
USAGE
}

die()
{
	echo "error: $*" >&2
	exit 1
}

[ "$#" -ge 1 ] && [ "$#" -le 3 ] || {
	usage >&2
	exit 2
}

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)
port=$1
package=${2:-$repo_root/build/mdk-bootloader/OpenPuck-MDK-bootloader-0.7.1-openpuck.zip}
baud=${3:-115200}

command -v adafruit-nrfutil >/dev/null 2>&1 || \
	die "adafruit-nrfutil is required"
[ -f "$package" ] || die "bootloader package not found: $package"

printf 'This writes only the OpenPuck-compatible bootloader package.\n'
printf 'Port: %s\n' "$port"
printf 'Package: %s\n' "$package"
adafruit-nrfutil dfu serial --package "$package" -p "$port" -b "$baud" \
	--singlebank
