# OpenPuck on the Nordic nRF52840 Dongle (PCA10059)

OpenPuck targets the Pro Micro nRF52840, which runs Adafruit's UF2 bootloader.
The Nordic nRF52840 Dongle (PCA10059) can run it too, but the release `.uf2` /
`.hex` won't work on it as-is. This document covers why, what this port
changes, and how to install it.

Verified on hardware: builds, flashes, pairs, settings survive a replug, and
the Dongle's bootloader still enters DFU mode afterwards.

## Why the release build doesn't work

| | Pro Micro (release build) | nRF52840 Dongle |
|---|---|---|
| Bootloader | Adafruit UF2, `0xF4000`+ | Nordic Open DFU, `0xE0000`+ |
| SoftDevice | S140 6.1.1 preinstalled | none |
| App start | `0x26000` | `0x1000` (no SoftDevice) |
| Install method | drag-and-drop `.uf2` | `nrfutil` DFU package |

- With no SoftDevice, the Dongle's bootloader places an app at `0x1000`. The
  release image is linked for `0x26000` and crashes anywhere else.
- The stock bootloader only accepts **signed** bootloader updates, so it can't
  be swapped for Adafruit's bootloader without a J-Link / SWD probe.
- OpenPuck writes to flash regions that the Dongle's bootloader owns:
  - InternalFS (config + bonds) at `0xED000`–`0xF3FFF`, hardcoded in the
    Adafruit core.
  - The fault black-box page at `0xE8000`.

  Running the release build, if it booted at all, would at best fault on the
  first config save and at worst erase the bootloader (SWD needed to recover).

## What the port changes

New build: `make build-pca10059` (board define `OPK_BOARD_PCA10059`). It
borrows the core's `adafruit:nrf52:mdbt50qrx` profile, the same one the Raytac
MDBT50Q-CX-40 port uses.

### Flash layout

| Region | Range |
|---|---|
| MBR + S140 6.1.1 | `0x00000`–`0x25FFF` |
| OpenPuck app | `0x26000`–`0xD7FFF` (build fails if it grows past this) |
| Fault black-box page | `0xD8000` |
| InternalFS | `0xD9000`–`0xDFFFF` |
| Nordic bootloader, MBR params, settings | `0xE0000`+ (untouched) |

### Files

- **`OpenPuck/board_config.h`**
  - `OPK_BOARD_PCA10059` disables the Adafruit-bootloader-only features
    (`OPK_HAS_ADAFRUIT_DFU 0`), same as the Raytac.
  - Adds `OPK_BB_ADDR`, the black-box page address per board: `0xD8000` on the
    Dongle, `0xE8000` everywhere else.
- **`OpenPuck/fault_diag.cpp`**: `BB_ADDR` now comes from `OPK_BB_ADDR`
  instead of a hardcoded `0xE8000`.
- **`OpenPuck/status_led.cpp`**: the wake-diagnostic LED is the Dongle's green
  LD1 on P0.06 (active-low). The Raytac keeps P0.08.
- **`Makefile`** gains these targets:
  - `build-pca10059` copies the core's `InternalFileSystem` library into
    `build/cache/pca10059-libs/` and patches it:
    - `LFS_FLASH_ADDR` `0xED000` → `0xD9000`
    - its write guard `BOOTLOADER_ADDR` `0xF4000` → `0xE0000`, so the
      library itself refuses any write into the bootloader

    User `--libraries` take priority over platform libraries, so this copy
    shadows the core's without modifying the installed core. The target
    also caps `upload.maximum_size` at 729088 bytes (`0xD8000 - 0x26000`).
  - `package-pca10059` builds an unsigned Secure DFU package. With
    `SOFTDEVICE_HEX` set, it bundles S140 6.1.1 (`--sd-req 0x00,0xB6`) so a
    fresh Dongle is provisioned in one flash. Without it, the package is
    app-only (`--sd-req 0xB6`).
  - `flash-pca10059` programs the package over `nrfutil device program`.
  - `deploy-pca10059` runs build, package and flash in sequence.
- **`docs/BUILD_AND_DEPLOY.md`**: new §4b with the short version of this
  document.

The normal Pro Micro build (`make build`) and the Raytac build
(`make build-raytac`) are unchanged and still compile. `make check` passes.

### Limitations (same as the Raytac port)

- The web configurator's **Firmware update** tab and its UF2/serial DFU
  buttons are rejected on this board. Update with the DFU workflow below.
- Factory reset (filesystem-only) and the serial `ERASE-ALL` command work
  normally.

## Install

### 1. Tools (macOS; one-time)

```sh
# arduino-cli + Adafruit nRF52 core
brew install arduino-cli
arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
arduino-cli core update-index
arduino-cli core install adafruit:nrf52

# nRF Util (Nordic's DFU tool)
mkdir -p ~/.local/bin
curl -fsSL -o ~/.local/bin/nrfutil \
  https://files.nordicsemi.com/artifactory/swtools/external/nrfutil/executables/universal-apple-darwin/nrfutil
chmod +x ~/.local/bin/nrfutil
export PATH="$HOME/.local/bin:$PATH"   # add to ~/.zshrc to keep it
nrfutil install nrf5sdk-tools device

# Only needed for `make check` before opening a PR
brew install llvm@18
```

On Linux or Windows, download nRF Util from
[Nordic](https://www.nordicsemi.com/Products/Development-tools/nRF-Util); the
rest is the same.

### 2. Get the S140 6.1.1 SoftDevice (first install only)

Either take `s140_nrf52_6.1.1_softdevice.hex` from Nordic's nRF5 SDK v15.3.0
(`components/softdevice/s140/hex/`), or extract it from a bootloader image the
Adafruit core already ships. That image holds MBR + S140 below `0x26000` and
Adafruit's bootloader above it; keep only the low part:

```sh
python3 - <<'EOF'
import glob, os
core = os.path.expanduser("~/Library/Arduino15/packages/adafruit/hardware/nrf52")
src = sorted(glob.glob(core + "/*/bootloader/raytac_mdbt50q_rx/*_s140_6.1.1.hex"))[-1]
base = 0
with open("s140_nrf52_6.1.1_softdevice.hex", "w") as out:
    for line in open(src):
        kind = int(line[7:9], 16)
        if kind == 4:
            base = int(line[9:13], 16) << 16
        elif kind == 0 and base + int(line[3:7], 16) >= 0x26000:
            continue
        out.write(line)
EOF
```

(Linux: the core lives under `~/.arduino15/packages/adafruit/...`.)

### 3. Build

```sh
./gen_version.sh          # embeds version + git hash
make build-pca10059
```

### 4. Put the Dongle in DFU mode

Plug it in and press the small **sideways RESET button** (it faces the USB
connector end, not the white SW1 button on top). The red LED pulses while the
bootloader is waiting.

### 5. Flash

First install (bundles the SoftDevice):

```sh
make package-pca10059 SOFTDEVICE_HEX=/path/to/s140_nrf52_6.1.1_softdevice.hex
make flash-pca10059
```

Later updates (app only; re-enter DFU mode first):

```sh
make deploy-pca10059
```

`nrfutil` prints a large "WARNING: not signed" banner when packaging. That is
expected: the Dongle's Open DFU bootloader accepts unsigned app and SoftDevice
packages.

### 6. Pair and use

From here it behaves like any OpenPuck:

1. Plug the Dongle and the controller (by USB-C cable) into the same PC with
   Steam running, and pair through Steam.
2. Configure it at <https://safijari.github.io/openpuck/> in Chrome or Edge.

The green LED stays dark normally. It flashes briefly when the puck sends a
USB remote-wake.

### Recovery

- **Bad config or bonds:** use the configurator's factory reset, or flash a
  recovery build:

  ```sh
  make build-pca10059 EXTRA_FLAGS="-DOPK_FACTORY_RESET=1"
  make package-pca10059 && make flash-pca10059
  ```

  then flash a normal build afterwards.
- **App won't boot:** press RESET to re-enter DFU and reflash. The bootloader
  is never written by this firmware.
