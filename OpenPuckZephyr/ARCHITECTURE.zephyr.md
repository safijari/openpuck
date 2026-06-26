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

### Functional / verified-as-compiling
- Build pipeline, persistence (LittleFS), config + bond load/save, the radio
  register layer, the RF protocol state machines, and all mode report-building
  logic compile under Zephyr.
- HFXO is started explicitly in `setup()` (the Arduino core did it implicitly);
  the WDT is driven directly via `NRF_WDT` (Zephyr's watchdog driver is left out
  to avoid double-ownership).

### NOT yet functional (the remaining work, none hardware-verified)
1. **USB is inert.** `usb_stub.cpp` satisfies the TinyUSB shim with safe no-ops
   (device never mounts, HIDs never ready). The device will **not enumerate**
   until the real Zephyr USBD backing is finished. The staged real backing lives
   in `usb_glue.cpp` and needs:
   - a devicetree overlay declaring N `zephyr,hid-device` nodes (the HID pool the
     shim binds to) plus CDC ACM, MSC, and a vendor (WebUSB) interface;
   - a `usbd_context` built per active mode (register exactly the classes that
     mode needs at init — see the concession below);
   - the USBD message callback wired to `opk_usb_msg` for mount/suspend state.
2. **WebUSB** vendor pipe (`Adafruit_USBD_WebUSB`) + **MS OS 2.0** descriptors —
   needed to preserve Windows/Chrome binding. Currently stubbed inert.
3. **XInput** is a custom TinyUSB class driver (`mode_xinput.cpp`, via the
   `device/usbd_pvt.h` shim). It compiles but needs a real Zephyr custom USBD
   class to enumerate; the report-building logic is intact.
4. **CDC console**: `serial_console` reads via the `opk_serial_*` weak hooks —
   wire them to a Zephyr CDC ACM (or RTT) endpoint.
5. **status_led** pin mapping: the Arduino code uses logical pin numbers; under
   the absolute-nRF-pin `digitalWrite` shim the B-pin (P0.15) needs an override.

### Behavioral concession (documented)
Zephyr's USBD descriptor set is fixed once the stack is enabled. The Arduino
firmware's **dynamic re-enumeration without reboot** (`usbReenumerate`, variable
connected-controller count) does not map; it collapses to **reboot-to-
reenumerate**, which was already the firmware's mode-switch path. Per-mode
interface sets are achieved by registering only that mode's classes at init.
