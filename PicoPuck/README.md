# PicoPuck

A Steam Controller 2 **puck emulator for the Raspberry Pi Pico W and Pico 2 W**.

Where the original OpenPuck is an nRF52840 dongle that talks to the controller
over Valve's proprietary 2.4 GHz ESB link, PicoPuck uses the Pico W's CYW43
radio to reach controllers over **Bluetooth** — both BLE and Classic — while
presenting the host with the *exact same* Valve puck USB device
(`28DE:1304`). Steam therefore sees a normal puck, and:

- A **Steam Controller 2** paired over its native Bluetooth mode is forwarded
  *transparently* — input reports go through verbatim and Steam's haptics /
  settings writes are relayed back to the controller, so it behaves just like a
  controller on a real puck.
- **Other Bluetooth controllers** (Xbox, DualShock/DualSense, Switch Pro,
  8BitDo, Stadia, …) are canonicalised into the puck's input model and, in Steam
  mode, presented as a controller with neutral trackpads/gyro.

Pairing, unpairing and scanning are driven from the same **WebUSB panel**
(`docs/index.html`) as the other OpenPuck devices.

> Status: under construction. See the phase checklist below.

## Building

You need the Arm GNU toolchain, CMake ≥ 3.13, and the Pico SDK **2.x** with its
`btstack`, `cyw43-driver`, `lwip`, `mbedtls`, and `tinyusb` submodules
initialised. Point `PICO_SDK_PATH` at it.

```sh
# Pico W (RP2040)
cmake -S PicoPuck -B PicoPuck/build-pico_w -DPICO_BOARD=pico_w
cmake --build PicoPuck/build-pico_w -j

# Pico 2 W (RP2350)
cmake -S PicoPuck -B PicoPuck/build-pico2_w -DPICO_BOARD=pico2_w
cmake --build PicoPuck/build-pico2_w -j
```

Outputs: `picopuck_pico_w.uf2` / `picopuck_pico2_w.uf2`. Flash by holding
BOOTSEL while plugging in, or from the panel's "reboot to bootloader" button.

There is also a `make picopuck` / `make picopuck-pico2w` convenience target in
the repo root `Makefile`.

## Phase checklist

- [x] **Phase 0** — scaffold: builds for both boards, heartbeat LED.
- [x] **Phase 1** — USB puck identity (4 HID slots + WebUSB) enumerates; panel connects.
- [x] **Phase 2** — generic BLE pad (Xbox) presented in Steam mode via a synthesized report.
- [x] **Phase 3** — Classic HID host + DualShock 4. *(DS5 / Switch Pro are drivers-in-waiting, see below.)*
- [x] **Phase 4** — Steam Controller 2 transparent forwarding over the Valve GATT service.
- [x] **Phase 5** — 4-slot model, flash bond persistence + auto-reconnect, scan-duty policy, firmware/bootloader button.

### Implemented in code (needs on-hardware validation)

The full architecture is in place and compiling for both boards, but has **not
been run on real hardware** yet. Everything below is correct-by-construction
(ported from OpenPuck / joypad-os byte layouts and the reversed SC2 protocol);
expect to iterate once flashed.

### Follow-on drivers / features (same interfaces, deferred)

- **DualSense (DS5)** — Classic, needs the CRC32-sealed output report and
  adaptive-trigger/touchpad handling; add a `drv_ds5` implementing `input_driver_t`.
- **Switch Pro** — Classic, needs the subcommand init handshake before it reports.
- **Battery** — read the BLE Battery Service (and the SC2's battery channel) into
  `g_battery[slot]` so Steam shows a level instead of the wired glyph.
- Concurrency/bandwidth and soak testing (see risks).

## Known risks / open questions

- **Concurrent-link ceiling.** BTstack pools are sized for four slots, but the
  CYW43 firmware's real concurrent Classic+BLE limit and ACL bandwidth under
  load are unverified; running inquiry+scan while controllers are connected is
  known to hurt throughput. The four USB slots are always enumerated regardless.
- **Latency.** A BLE connection interval floor (~7.5 ms) delivers input slower
  than the nRF puck's RF poll; verbatim on-air timestamps keep gyro integration
  correct but feel is to be compared against a real puck.
- **Windows descriptor cache.** Same VID/PID as the nRF puck with a different
  interface layout — `bcdDevice` is bumped to `0x03xx` to force a fresh cache
  entry; test both plug orders.
- **BTstack licensing.** Covered by the SDK's `LICENSE.RP` grant only on
  Raspberry Pi silicon; see `NOTICE`.
