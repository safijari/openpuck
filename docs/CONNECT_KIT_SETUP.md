# Running OpenPuck on the MakerDiary nRF52840 Connect Kit

Native support for the **MakerDiary nRF52840 Connect Kit** — a reliability-focused alternative to the
usual Pro Micro / Feather. It has **two crystals** (a 32 MHz HFXO *and* a 32.768 kHz LFXO — most cheap
Pro Micro clones omit the LFXO), a clean buck-boost supply, and a **U.FL external-antenna** option.

OpenPuck runs on it as a **native no-SoftDevice (nosd) build**, flashed by plain **drag-and-drop** —
**no SWD probe, no SoftDevice, no bootloader swap.** Get the **U.FL variant** if you want an external antenna.

## Quick start

1. **Install the toolchain** (once): arduino-cli + the Adafruit nRF52 core — see
   [BUILD_AND_DEPLOY.md](./BUILD_AND_DEPLOY.md) §2–3.
2. **Build:**
   ```sh
   ./gen_version.sh        # optional: bakes version/git provenance into the build
   make build-connectkit   # -> build/connectkit/OpenPuck-connectkit.uf2
   ```
3. **Flash:** double-tap the **RST** button — the board mounts as a **`UF2BOOT`** drive. Copy
   `build/connectkit/OpenPuck-connectkit.uf2` onto it. It reboots into OpenPuck and enumerates as the
   Steam puck.
4. **Pair** the controller (see [Pairing](#pairing)).

Later firmware updates can also be done from the
[WebUSB configurator](https://safijari.github.io/openpuck/)'s **Firmware update** tab, like any OpenPuck.

## Why this board

- **Reliability:** a real 32 MHz HFXO crystal (the radio's timing reference) **and** a 32.768 kHz LFXO —
  the two things flaky no-name clones most often get wrong.
- **External antenna:** the U.FL variant closes most of the ~20 dB the firmware notes for a bare
  PCB-trace antenna, getting you into real-puck range.
- **Passive +8 dBm RF front end** (no PA/LNA) — exactly what OpenPuck drives.

## How it works (the nosd approach)

The Connect Kit's stock UF2 bootloader ships **with no SoftDevice** and hands the application control at
**`0x1000`**. OpenPuck never uses a SoftDevice anyway, so rather than swap the bootloader it runs
**natively at `0x1000`, no SoftDevice**. All of this lives in the in-repo **vendored Arduino platform**
at `arduino/hardware/openpuck/nrf52/` (which references the installed Adafruit nRF52 core) — **OpenPuck's
own source is untouched.**

Three board-specific pieces make it work, all in the vendored platform:

1. **Link at `0x1000`, no SoftDevice** — a variant linker script (`nrf52840_connectkit_nosd.ld`) plus a
   `boards.txt` `build.flags.nrf` that drops `-DSOFTDEVICE_PRESENT`.
2. **Set `VTOR` early** — `variant.cpp` sets `SCB->VTOR = 0x1000` in a `constructor`, *before*
   `main()`/`init()` enables the first interrupt. (The bootloader doesn't point the vector table at the
   app; doing it later, e.g. in `initVariant()`, reset-loops.)
3. **Direct-NVMC flash** — the Adafruit `InternalFileSystem` flash layer calls the SoftDevice flash SVCs
   (`sd_flash_write` / `sd_flash_page_erase`), which have no handler on a nosd chip and crash on the first
   bond write. Built with `-DSVCALL_AS_NORMAL_FUNCTION`, the variant provides NVMC implementations of
   those (bounds-checked to `[0x1000, 0xF4000)` so they can never touch the MBR or bootloader).

Official flash map (MakerDiary `nrf52840_partition_uf2_nosd.dtsi`):

| Region | Range |
|---|---|
| MBR | `0x00000`–`0x01000` |
| **Application** (OpenPuck) | `0x01000`–`0xD4000` |
| **Storage** (LittleFS: config + bonds) | `0xD4000`–`0xF4000` |
| UF2 bootloader | `0xF4000`–`0x100000` |

## Pairing

Plug both the Connect Kit and the controller into the same machine via **data** USB-C, with **Steam running**.

- Pair into **slot 2** (leaves your existing slot-1 puck intact): turn the controller off (**Steam + Y**),
  then power it on holding **LB + A + Steam** (a different chime confirms slot 2), then pair in Steam
  (auto-popup, or **Settings → Controllers → Add Controller**).
- Slots: **RB + A + Steam** = slot 1, **LB + A + Steam** = slot 2.

Confirm with an input test in Steam (buttons / sticks / gyro / trackpads).

## Antenna

Fit a 2.4 GHz antenna to the **U.FL** connector (U.FL→SMA pigtail). Do **not** add an external PA/LNA —
OpenPuck only drives the nRF's internal +8 dBm, and an unpowered external LNA would hurt sensitivity.

## Recovery / troubleshooting

- **Un-brickable by flashing:** drag-drop writes only the app region; the MBR and bootloader are never
  touched. A bad app just means double-tap **RST** → `UF2BOOT` → drag a good `.uf2`.
- **`make build-connectkit` prints "adafruit-nrfutil not found":** harmless — that's only arduino-cli's
  optional DFU-zip step; the `.uf2` is still produced (the target tolerates it).
- **Dev serial log:** the board can log over UART on **P1.09 @ 115200** if you re-add a debug logger to
  the variant; wire adapter **RX ← P1.09**, **GND ← GND**, adapter at **3.3 V**.

## Reference

- **FQBN:** `openpuck:nrf52:makerdiary_connectkit`
- **Build:** `make build-connectkit` — or `make build-connectkit-recovery` for a one-time factory-reset image.
- **Bootloader drive:** `UF2BOOT` (enter: double-tap **RST**, or hold **USER** while plugging in).
- **Pins:** green LED P1.15; RGB red/green/blue P1.10/P1.11/P1.12 (OpenPuck leaves them off); USER button P1.00; reset P0.18.
- **Official board docs:** <https://wiki.makerdiary.com/nrf52840-connectkit/>
