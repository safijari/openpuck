#!/usr/bin/env bash
set -euo pipefail

PINNED_COMMIT=c87ea51b86b96f9b19458abfc6b37d4cb52e160b
BUILD_LABEL=0.7.1-openpuck

usage()
{
	cat <<'USAGE'
Usage: build_mdk_bootloader.sh [--work-dir DIR] [--arm-gcc-prefix PREFIX]
USAGE
}

die()
{
	echo "error: $*" >&2
	exit 1
}

need()
{
	command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)
work_dir="$repo_root/build/mdk-bootloader"
arm_gcc_prefix="${ARM_GCC_PREFIX:-}"

while [ "$#" -gt 0 ]; do
	case "$1" in
	--work-dir)
		[ "$#" -ge 2 ] || die "--work-dir requires a value"
		work_dir=$2
		shift 2
		;;
	--arm-gcc-prefix)
		[ "$#" -ge 2 ] || die "--arm-gcc-prefix requires a value"
		arm_gcc_prefix=$2
		shift 2
		;;
	-h | --help)
		usage
		exit 0
		;;
	*)
		die "unknown argument: $1"
		;;
	esac
done

need git
need make
need python3
need adafruit-nrfutil
need sha256sum
python3 -c 'import intelhex' >/dev/null 2>&1 || \
	die "Python package intelhex is required"

resolve_arm_prefix()
{
	local gcc

	if [ -n "$arm_gcc_prefix" ]; then
		printf '%s\n' "$arm_gcc_prefix"
		return
	fi

	if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
		gcc=$(command -v arm-none-eabi-gcc)
		printf '%s\n' "${gcc%gcc}"
		return
	fi

	gcc=$(
		for root in \
			"$HOME/.arduino15/packages/adafruit/tools/arm-none-eabi-gcc" \
			"$HOME/.arduino15/packages/arduino/tools/arm-none-eabi-gcc"; do
			[ -d "$root" ] || continue
			find "$root" -type f \
				-path '*/bin/arm-none-eabi-gcc' -print
		done | sort -V | tail -n1
	)
	[ -n "$gcc" ] || die "arm-none-eabi-gcc was not found"
	printf '%s\n' "${gcc%gcc}"
}

arm_gcc_prefix=$(resolve_arm_prefix)
patch_reset="$script_dir/0001-Makerdiary-reset-UICR.patch"
patch_handoff="$script_dir/0002-OpenPuck-app-handoff.patch"
verify="$repo_root/tools/verify_mdk_bootloader_hex.py"
repo="$work_dir/Adafruit_nRF52_Bootloader"

mkdir -p "$work_dir"
if [ ! -d "$repo/.git" ]; then
	git clone https://github.com/adafruit/Adafruit_nRF52_Bootloader.git "$repo"
fi

git -C "$repo" fetch --all --tags
git -C "$repo" reset --hard
git -C "$repo" clean -fdx
git -C "$repo" checkout --detach "$PINNED_COMMIT"
git -C "$repo" submodule sync
git -C "$repo" submodule update --init --force lib/nrfx lib/tinyusb lib/uf2

git -C "$repo" apply --check --ignore-whitespace "$patch_reset"
git -C "$repo" apply --ignore-whitespace "$patch_reset"
git -C "$repo" apply --check --ignore-whitespace "$patch_handoff"
git -C "$repo" apply --ignore-whitespace "$patch_handoff"

# The upstream Makefile packs the numeric 0.7.1 prefix while the complete
# marker remains visible to UF2 hosts as 0.7.1-openpuck.
make -C "$repo" BOARD=mdk_nrf52840_dongle GIT_VERSION="$BUILD_LABEL" \
	CROSS_COMPILE="$arm_gcc_prefix"

boot_hex="$repo/_build/build-mdk_nrf52840_dongle/mdk_nrf52840_dongle_bootloader-$BUILD_LABEL.hex"
out_zip="$work_dir/OpenPuck-MDK-bootloader-$BUILD_LABEL.zip"

[ -f "$boot_hex" ] || die "expected bootloader HEX missing: $boot_hex"
python3 "$verify" "$boot_hex"
rm -f "$out_zip"
adafruit-nrfutil dfu genpkg --dev-type 0x0052 --dev-revision 52840 \
	--sd-req 0xB6 --bootloader "$boot_hex" "$out_zip"

sha256sum "$boot_hex" "$out_zip"
printf 'Bootloader HEX: %s\n' "$boot_hex"
printf 'Bootloader DFU: %s\n' "$out_zip"
