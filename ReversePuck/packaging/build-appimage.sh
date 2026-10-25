#!/usr/bin/env bash
# Builds build/reversepuck-appimage/ReversePuck-<version>-x86_64.AppImage, the Steam Deck forwarder as one
# file you can add to Steam as a non-Steam game and launch from Game Mode. No uv, no network, no sudo.
#
# The AppImage is a prebuilt portable Python (python-appimage, manylinux2014) plus the manylinux wheels for
# pygame, pyserial and pyusb, plus this folder's app, packed as squashfs behind the standard AppImage type-2
# runtime. Works on macOS or Linux:
#   - with squashfs-tools installed (mksquashfs/unsquashfs; `brew install squashfs` on macOS)
#   - or on x86_64 Linux without them, using appimagetool instead (e.g. on a Steam Deck).
# Needs curl and python3 with pip (wheels are fetched for the target platform, not the host). Downloads
# are cached in build/reversepuck-appimage/cache.
#
#   ReversePuck/packaging/build-appimage.sh            # from the repo root
#   VERSION=1.2.3 ReversePuck/packaging/build-appimage.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$(dirname "$HERE")"
REPO="$(dirname "$APP")"
BUILD="$REPO/build/reversepuck-appimage"
CACHE="$BUILD/cache"
APPDIR="$BUILD/AppDir"

PY_URL="${PY_URL:-https://github.com/niess/python-appimage/releases/download/python3.12/python3.12.15-cp312-cp312-manylinux2014_x86_64.AppImage}"
# uruntime instead of the stock type2 runtime: Steam starts Game Mode games with no_new_privs, which stops
# the setuid fusermount from mounting, and the stock runtime then just exits ("Cannot mount AppImage")
# unless the launch options carry APPIMAGE_EXTRACT_AND_RUN=1. uruntime's default policy tries FUSE and
# falls back to extract-and-run by itself. Pinned by version and SHA-256 (GitHub's release asset digest).
RUNTIME_URL="${RUNTIME_URL:-https://github.com/VHSgunzo/uruntime/releases/download/v0.8.1/uruntime-appimage-squashfs-x86_64}"
RUNTIME_SHA256="${RUNTIME_SHA256:-2153e68d63f570c7ebe3d1f38df0fa16814f34d839c5cce2ee42667e327708e8}"
TOOL_URL="${TOOL_URL:-https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage}"
WHEELS="${WHEELS:-pygame==2.6.1 pyserial==3.5 pyusb==1.3.1}"
PYTHON="${PYTHON:-python3}"

VERSION="${VERSION:-$(git -C "$REPO" describe --tags --always 2>/dev/null || echo dev)}"
OUT="$BUILD/ReversePuck-$VERSION-x86_64.AppImage"

fetch() {
	local url="$1" dest="$CACHE/$(basename "$1")"
	if [ ! -s "$dest" ]; then
		echo "Downloading $(basename "$url")" >&2
		curl -fL --retry 3 -o "$dest.part" "$url"
		mv "$dest.part" "$dest"
	fi
	echo "$dest"
}

# An AppImage's squashfs starts right after its ELF runtime: end of the section header table.
elf_size() {
	"$PYTHON" -c '
import struct, sys
d = open(sys.argv[1], "rb").read(64)
shoff, = struct.unpack_from("<Q", d, 0x28)
shentsize, shnum = struct.unpack_from("<HH", d, 0x3A)
print(shoff + shentsize * shnum)' "$1"
}

have_squashfs() { command -v mksquashfs >/dev/null && command -v unsquashfs >/dev/null; }

check_sha256() {
	"$PYTHON" -c '
import hashlib, sys
got = hashlib.sha256(open(sys.argv[1], "rb").read()).hexdigest()
if got != sys.argv[2]:
    sys.exit("%s: SHA-256 %s, expected %s" % (sys.argv[1], got, sys.argv[2]))' "$1" "$2"
}

mkdir -p "$CACHE"
rm -rf "$APPDIR"

PY_APPIMAGE="$(fetch "$PY_URL")"
echo "Unpacking Python"
if have_squashfs; then
	unsquashfs -q -n -o "$(elf_size "$PY_APPIMAGE")" -d "$APPDIR" "$PY_APPIMAGE" >/dev/null
elif [ "$(uname -s)" = Linux ]; then
	chmod +x "$PY_APPIMAGE"
	(cd "$BUILD" && rm -rf squashfs-root && "$PY_APPIMAGE" --appimage-extract >/dev/null)
	mv "$BUILD/squashfs-root" "$APPDIR"
else
	echo "Install squashfs-tools first (macOS: brew install squashfs)." >&2
	exit 1
fi

echo "Trimming Python"
PYLIB="$APPDIR/opt/python3.12/lib/python3.12"
SITE="$PYLIB/site-packages"
# pygame draws the UI, so Tk goes too
rm -rf "$APPDIR"/AppRun "$APPDIR"/*.desktop "$APPDIR"/*.png "$APPDIR"/.DirIcon \
	"$APPDIR"/usr/share/applications "$APPDIR"/usr/share/metainfo "$APPDIR"/usr/share/icons \
	"$APPDIR"/usr/share/tcltk "$PYLIB"/tkinter "$PYLIB"/lib-dynload/_tkinter* \
	"$PYLIB"/{idlelib,ensurepip,lib2to3,pydoc_data,turtledemo,test,unittest/test} \
	"$SITE"/{pip,pip-*,setuptools,setuptools-*,_distutils_hack,distutils-precedence.pth} \
	"$APPDIR"/opt/python3.12/bin/pip*
find "$APPDIR" -name __pycache__ -type d -prune -exec rm -rf {} +

echo "Adding wheels: $WHEELS"
WHEELDIR="$CACHE/wheels"
mkdir -p "$WHEELDIR"
# shellcheck disable=SC2086
"$PYTHON" -m pip download --quiet --disable-pip-version-check --only-binary=:all: \
	--platform manylinux2014_x86_64 --platform manylinux_2_17_x86_64 \
	--python-version 3.12 --implementation cp --abi cp312 --abi abi3 --abi none \
	--dest "$WHEELDIR" $WHEELS
# A wheel is a zip laid out for site-packages; *.data/{purelib,platlib} also belong there, the rest
# (headers, scripts) isn't needed. Only the wheels named in WHEELS are unpacked, not stale cache entries.
"$PYTHON" - "$SITE" "$WHEELDIR" $WHEELS <<'PY'
import os, re, shutil, sys, zipfile
site, wheeldir, wanted = sys.argv[1], sys.argv[2], sys.argv[3:]
def norm(s):
    return re.sub(r"[-_.]+", "_", s).lower()
for spec in wanted:
    name, version = spec.split("==")
    # wheel file names are <name>-<version>-<tags>.whl
    matches = [w for w in os.listdir(wheeldir) if w.endswith(".whl")
               and w.split("-")[:2] == [norm(name), version]]
    if len(matches) != 1:
        sys.exit("expected one wheel for %s in %s, found %s" % (spec, wheeldir, matches))
    with zipfile.ZipFile(os.path.join(wheeldir, matches[0])) as z:
        z.extractall(site)
    print("  " + matches[0])
for entry in os.listdir(site):
    if entry.endswith(".data"):
        data = os.path.join(site, entry)
        for sub in ("purelib", "platlib"):
            src = os.path.join(data, sub)
            if os.path.isdir(src):
                for item in os.listdir(src):
                    shutil.move(os.path.join(src, item), site)
        shutil.rmtree(data)
PY
# pygame's examples, docs and tests are a third of its size
rm -rf "$SITE"/pygame/{examples,docs,tests}

echo "Adding the app"
SHARE="$APPDIR/usr/share/reversepuck"
mkdir -p "$SHARE" "$APPDIR/usr/share/applications" "$APPDIR/usr/share/icons/hicolor/256x256/apps"
cp "$APP"/{openctrl,ui,frame,input_source,scpair}.py "$SHARE/"
cp "$HERE/AppRun" "$APPDIR/AppRun"
chmod 755 "$APPDIR/AppRun"
cp "$HERE/reversepuck.desktop" "$APPDIR/"
cp "$HERE/reversepuck.desktop" "$APPDIR/usr/share/applications/"
"$PYTHON" "$HERE/make_icon.py" "$APPDIR/reversepuck.png"
cp "$APPDIR/reversepuck.png" "$APPDIR/usr/share/icons/hicolor/256x256/apps/reversepuck.png"
ln -sf reversepuck.png "$APPDIR/.DirIcon"

echo "Packing $OUT"
rm -f "$OUT"
RUNTIME="$(fetch "$RUNTIME_URL")"
check_sha256 "$RUNTIME" "$RUNTIME_SHA256"
if have_squashfs; then
	mksquashfs "$APPDIR" "$BUILD/app.squashfs" -noappend -all-root -no-xattrs \
		-comp zstd -b 1M -quiet >/dev/null
	cat "$RUNTIME" "$BUILD/app.squashfs" >"$OUT"
	rm -f "$BUILD/app.squashfs"
else
	TOOL="$(fetch "$TOOL_URL")"
	chmod +x "$TOOL"
	ARCH=x86_64 APPIMAGE_EXTRACT_AND_RUN=1 "$TOOL" --no-appstream --runtime-file "$RUNTIME" \
		"$APPDIR" "$OUT"
fi
chmod 755 "$OUT"
echo "Built $OUT ($(du -h "$OUT" | cut -f1))"
