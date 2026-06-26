# OpenPuck — Zephyr RTOS port

This is the Zephyr port of the OpenPuck firmware (originally Arduino /
Adafruit-nRF52 / TinyUSB, in `../OpenPuck`). It targets mainline Zephyr
(v4.4.99) on `adafruit_feather_nrf52840/nrf52840/uf2`.

## Build

```sh
./zbuild.sh                 # uses ~/zephyrproject + ~/zephyr-sdk-1.0.1 via the venv
# or, with env set up:
west build -b adafruit_feather_nrf52840/nrf52840/uf2 OpenPuckZephyr
```

Output `build/zephyr/zephyr.uf2` flashes via the existing double-tap-reset DFU
procedure (the `uf2` board variant reserves the Adafruit bootloader region and
uses app base 0x26000).

## Migration strategy: compatibility shims

The reverse-engineered radio/protocol/mode logic is the value of this firmware
and the part most dangerous to rewrite. So instead of rewriting it, the port
keeps those ~8000 lines near-verbatim and supplies a thin compatibility layer in
`src/compat/`:

| Shim | Replaces | Backing |
|---|---|---|
| `Arduino.h` / `arduino_compat.cpp` | Arduino core | `micros()` via Cortex-M DWT cycle counter; `millis()`/`delay()` via kernel; GPIO via `nrf_gpio`; `Serial` via console; `NRF_*` registers + CMSIS via nrfx MDK (`<nrfx.h>`) |
| `Adafruit_LittleFS.h` / `InternalFileSystem.h` / `littlefs_shim.cpp` | Adafruit LittleFS | Zephyr `fs` (LittleFS) mounted on `storage_partition` |
| `Adafruit_TinyUSB.h` | TinyUSB / Adafruit USBD | Zephyr USBD (see USB status below) |

The `NRF_RADIO`/`NRF_WDT`/`NRF_FICR`/`NRF_CLOCK` register code ports essentially
unchanged because Zephyr's hal_nordic ships the same Nordic MDK. `CONFIG_BT`,
`CONFIG_IEEE802154`, and the 802.15.4 radio driver are disabled so the RADIO
peripheral is free for the bare-metal protocol.

## Status

**Compiles + links + produces a UF2**, with the real `setup()`/`loop()` entry
(`main.cpp`, ported from `OpenPuck.ino`). Every module is in the build:
radio, rf_link, rf_diag, triton, gamepad_util, haptics, controllers, puck_hid,
all six modes (lizard, xinput, switch_hori, switch_pro, ps5, hidgyro),
webusb_config, serial_console, usb_mount, wake_hid, config, bonds.

### Functional (compiles + linked into the UF2; hardware-verification pending)
- Build pipeline, persistence (LittleFS), config + bond load/save, the radio
  register layer, the RF protocol state machines, and all mode report-building
  logic.
- HFXO started explicitly in `setup()`; WDT driven directly via `NRF_WDT`.
- **USB is wired (not inert):** a real `usbd_context` (`usb_device_setup.cpp`)
  brings the stack up via the TinyUSB shim's `attach()`. It registers, per active
  mode, only the classes that mode used:
  - **HID** — a 5-node `zephyr,hid-device` pool (`app.overlay`); the shim
    (`usb_glue.cpp`) binds `Adafruit_USBD_HID` objects in `begin()` order,
    `sendReport`→`hid_device_submit_report`, get/set/output reports dispatched to
    the modes. Covers puck/lizard slots, switch (hori/pro), ps5, hidgyro, the
    wake mouse, and the Xbox right-pad mouse.
  - **CDC ACM** — the board console; `serial_cdc_glue.cpp` wires the debug
    console's input + output to it.
  - **WebUSB** — a custom vendor class (`usb_webusb_class.cpp`) bridged to byte
    rings, plus a BOS WebUSB platform capability + landing-page URL.
  - **XInput** — a custom vendor class (`usb_xinput_class.cpp`): the FF/5D/01
    interface + magic blob + interrupt IN/OUT; `mode_xinput` feeds the 20-byte
    report and decodes the OUT rumble packet.
- Per-mode interface selection (the reboot-to-reenumerate concession) is done by
  blocklisting the unused class instances in `usb_device_setup.cpp`, which also
  keeps within the nRF52840's 8 IN-endpoint budget.

### Remaining / to confirm on hardware (none hardware-verified here)
1. **Everything USB needs real-hardware verification** — enumeration, the per-mode
   class-instance names (`hid_0..hid_4`, `cdc_acm_0`, `opk_webusb_0`,
   `opk_xinput_0`), endpoint assignment, and the per-mode endpoint budget.
2. **XInput is single-controller.** Multi-controller XInput (one interface per
   connected pad, as the Arduino dynamic-mount did) is a follow-up.
3. **Dynamic mount.** The variable connected-controller count without reboot is
   collapsed to a fixed per-mode set + reboot-to-reenumerate (the concession
   below). The `usb_mount` watcher is compiled but its re-enumeration is inert.
4. **MS OS 2.0 / WinUSB** auto-binding for WebUSB on Windows is not added (the
   WebUSB BOS + URL are). The browser panel works where the OS lets a page claim
   a vendor interface; Windows may need a manual WinUSB association.
5. **Wake-on-Windows nuance:** the boot-mouse wake interface is a generic HID
   node (`protocol-code = "none"`); for Windows to arm it as a wake source the
   wake node likely needs `protocol-code = "mouse"`.

### Behavioral concession (documented)
Zephyr's USBD descriptor set is fixed once the stack is enabled. The Arduino
firmware's **dynamic re-enumeration without reboot** (`usbReenumerate`, variable
connected-controller count) does not map; it collapses to **reboot-to-
reenumerate**, which was already the firmware's mode-switch path. Per-mode
interface sets are achieved by registering only that mode's classes at init.
