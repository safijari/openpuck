# OpenPuck Architecture

This document is the map of the firmware. It explains what the device *is*, how the code is laid out, how
data flows through it, and how to extend it. If you only read one thing before touching the code, read this.

## What the device is

OpenPuck turns an **nRF52840 Pro Micro** into a clone of the Steam Controller 2 "puck" — the little USB radio
dongle the controller pairs with. It does three jobs at once:

1. **Speaks the puck's RF protocol** to a real Steam Controller 2 ("Triton") over a bare-metal nRF radio (no
   SoftDevice, no BLE stack). It bonds controllers, transmits the host frame they reconnect on, and polls them
   for input at ~250 Hz.
2. **Presents a USB personality** to the host PC/console. The same controller input can be re-dressed as the
   Valve puck (for Steam), an Xbox 360 pad, a Switch controller, a PS5 DualSense, or a DS4 — selected at boot.
3. **Exposes config surfaces** — a WebUSB channel for the browser panel and a CDC serial console for debugging
   and the protocol reverse-engineering tooling.

It runs as a **single-threaded `loop()`** plus **USB interrupt callbacks**. There is no RTOS task switching in
our code; concurrency is cooperative. The only preemption is the USB stack's ISR (which calls our HID report
callbacks) and the radio's hardware events (which we poll, never interrupt on).

## The big picture

```
                 ┌──────────────────────── host PC / console ────────────────────────┐
                 │  USB: one personality at a time (puck / xbox / switch / ps5 / ds4) │
                 └───────────────▲─────────────────────────────────▲──────────────────┘
                                 │ HID reports                      │ WebUSB / CDC
                                 │                                  │
        ┌────────────────────────┴─────────────┐        ┌──────────┴───────────┐
        │  controllers.h  (IController vtable)  │        │ webusb_config        │
        │  mode_*.cpp  — one per personality    │        │ serial_console       │
        └───────▲───────────────────▲───────────┘        └──────────┬───────────┘
   onReport45() │                   │ task()                        │ tunables / commands
        (push)  │                   │ (stream)                      ▼
        ┌───────┴───────────────────┴───────────────────────────────────────────┐
        │  rf_link.cpp  — operational puck protocol                              │
        │   • host-frame beacon  • connected poll (E7/E3)  • decode 0xF1 → g_in  │
        │   • mode-switch chord  • remote wakeup  • QoS channel hop  • stats     │
        └───────▲──────────────────────────────────────────▲────────────────────┘
                │ rfConfig / rfConnTx                       │ relay (haptics.cpp)
        ┌───────┴────────────┐                     ┌────────┴──────────┐
        │  radio.cpp         │   rf_diag.cpp ─────▶│  (RE/calibration  │
        │  nRF52 RADIO HW    │   (capture/sniff/   │   tooling, isolated)
        └────────────────────┘    replay/sweep)    └───────────────────┘

   shared state:  config (modes+tunables) · identity · bonds (slots+link) · triton (g_in) · gamepad_util
```

The **central data flow** is one direction for input:

> radio RX → `rf_link` decodes report `0x45` into **`g_in`** (a `PuckInput` struct) → the active
> `IController` reads `g_in` and emits a host HID report.

and the reverse for haptics/config:

> host OUTPUT report / WebUSB / CDC → queued into the haptic relay or a tunable → `rf_link` flushes the relay
> to the controller inside the poll cadence.

## Module layout (`OpenPuck/`)

The Arduino build compiles every `.cpp`/`.h` in the sketch folder; the build command is unchanged (it still
just points at the `OpenPuck` directory). Modules are layered low → high:

| File | Responsibility |
|---|---|
| **`OpenPuck.ino`** | Entry point only: `setup()` builds the USB presentation for the persisted mode + arms the watchdog; `loop()` pumps each subsystem. |
| `config.{h,cpp}` | USB-mode definitions (`MODE_*`), persisted settings (`cfg.bin`), all runtime tunables, mode-switch persistence policy. |
| `identity.{h,cpp}` | Unique device serial from FICR DEVICEID; the puck `0x83` attribute blob. |
| `bonds.{h,cpp}` | The four bond slots (`g_slot`), bond persistence (`bonds.bin`), and the live link state (`g_connSlot`, `g_connReplyMs`). |
| `radio.{h,cpp}` | Bare-metal nRF52 `RADIO` register layer: PHY/CRC/whitening/address config (all live-tunable), `rfConfig()`/`rfSetAddr()`, the `rfrx`/`rftx` DMA buffers. |
| `triton.{h,cpp}` | The controller's native input: report `0x45` button masks (`TB_*`) + decoders, and the shared decoded-input struct **`g_in`**. |
| `gamepad_util.{h,cpp}` | Cross-mode helpers for building host reports: stick rescaling, Steam-trackpad → touch mapping, PlayStation-layout button packers. |
| `haptics.{h,cpp}` | Host→controller haptic relay queue + the watchdogs that stop a latched buzz; the OUTPUT-report capture ring. |
| **`controllers.{h,cpp}`** | The `IController` abstraction + the `mode → singleton` registry. |
| `puck_hid.{h,cpp}` | **Steam/Lizard** personality: the puck HID slot interfaces, the feature command channel (bond read/write, attributes), the auto-lizard decision, and the USB connection presentation. |
| `mode_lizard.{h,cpp}` | The lizard keyboard+mouse mapping (rides on the puck interface; not a standalone controller). |
| `mode_xinput.{h,cpp}` | **Xbox** personality: a custom TinyUSB XInput class driver + right-pad mouse + rumble relay. |
| `mode_switch_hori.{h,cpp}` | **Switch HORIPAD** personality (console-friendly, no handshake). |
| `mode_switch_pro.{h,cpp}` | **Switch Pro** personality: the full Nintendo USB handshake/subcommand state machine + SPI calibration + gyro. |
| `mode_ps5.{h,cpp}` | **PS5 DualSense** personality: gyro + split trackpad. |
| `mode_hidgyro.{h,cpp}` | **DS4-layout** generic HID gyro personality (motion-aware PC games). |
| `rf_link.{h,cpp}` | The operational puck protocol: host-frame beacons, connected-mode poll loop, `0xF1` decode + dispatch, chord detection, remote wakeup, QoS hopping, stats. |
| `rf_diag.{h,cpp}` | RF reverse-engineering / calibration tooling: raw capture, CRC-validating config sweeps, frame replay, address listen, scan-then-respond, live-session sniffer. Not used in normal operation. |
| `webusb_config.{h,cpp}` | The WebUSB binary config channel for the browser panel. |
| `serial_console.{h,cpp}` | The CDC single-letter debug command line. |

## The controller abstraction

`IController` (in `controllers.h`) is the seam that makes each USB personality a self-contained file. Exactly
one controller is active per boot, chosen by `g_usbMode` via `controllerFor()` and stored in `g_active`.

```cpp
class IController {
  virtual void begin();                                              // setup(): register USB interface(s) + IDs
  virtual void onReport45(const uint8_t* rep, bool fresh, uint8_t bodyTlen);  // RF input arrived (PUSH modes)
  virtual void task();                                               // loop(): streaming emit / handshake drains
  virtual bool isPuck() const;                                       // Steam/Lizard keep the boot CDC composite
};
```

Two emission styles:

- **Push modes** (puck/lizard, xbox) build and send their host report inside `onReport45()`, driven by each
  decoded RF frame. They care about `fresh` (was the report new?) and the raw report bytes.
- **Stream modes** (switch hori/pro, ps5, ds4) ignore `onReport45()` and instead emit at a fixed ~250 Hz
  cadence from `task()`, reading the latest `g_in`. This decouples the USB report rate from the RF rate, which
  is what those hosts expect.

`setup()` owns the shared USB lifecycle (detach, keep-or-clear the CDC composite based on `isPuck()`, set the
per-mode serial suffix, attach); `begin()` only adds the interfaces specific to that controller.

### Adding a new personality

1. Create `mode_foo.{h,cpp}` defining `class FooController : public IController` and a singleton
   `extern FooController g_fooCtl;`.
2. Implement `begin()` (set `USBDevice` VID/PID/strings, register your HID/vendor interface) and either
   `onReport45()` (push) or `task()` (stream) reading `g_in`.
3. Add a `MODE_FOO` constant in `config.h` (bump `MODE_MAX`), a `MODE_SUFFIX` letter in `OpenPuck.ino`, and a
   `case MODE_FOO: return &g_fooCtl;` in `controllerFor()` (`controllers.cpp`).

Nothing in `rf_link`, `radio`, or the other modes needs to change — `g_in` is your input contract.

## Shared input state: `g_in`

`rf_link` decodes the controller's report `0x45` into a single `PuckInput g_in` (`triton.h`): buttons, sticks,
triggers, both trackpad coordinates, and IMU (accel + gyro). Every controller reads from this one struct. (The
pre-refactor firmware copied the same IMU/stick/button values into three separate per-mode global sets; that
duplication is gone.)

The raw report bytes are still passed to `onReport45()` for the modes that forward them verbatim (the Steam
path forwards the puck's raw `0x45` body untouched so Steam does its own trackpad smoothing).

## The RF protocol, briefly

Full details are in `docs/PROTOCOL.md`; the register-level constants and their provenance are commented in
`radio.cpp` and `rf_link.cpp`. The essentials:

- **PHY/framing** (decoded from real-puck capture): `Ble_2Mbit`, big-endian, whitening off, 5-byte address
  `"ibex"`, CRC16 `0x11021`/`0xFFFF` (address included). On-air address is the bit-reversed stored bytes.
- **Bonding/reconnect**: we transmit a *host frame* on the rendezvous address (channel 2) carrying the bonded
  controller's UUIDs plus the session base/prefix/channel we want it to use. The controller filters by UUID
  and connects on the clean session channel (default 18).
- **Connected poll**: we are the PTX/master. We announce host-awake (`E7 00 00`), then each poll cycle send
  `E3` + a GET-report-`0x45` sub-TLV; the controller replies with a `0xF1` input report containing TLVs
  (type 6 = the HID report `0x45`). Cycling the ESB PID on the poll is what drains the controller's report
  queue at full rate (~400 new reports/s instead of ~60).
- **Haptics** ride back as a SET sub-TLV inside the poll (`haptics.cpp` queues, `rf_link` flushes).

## Timing & concurrency model

- One cooperative `loop()`. Each subsystem's per-loop hook (`g_active->task()`, `rfLinkTask()`, `hapticTask()`,
  `rfDiagTask()`, `webusbPoll()`, `serialConsolePoll()`) must be **non-blocking** — anything that spins risks
  starving USB servicing.
- The RF poll is **busy-wait bounded**: every radio wait (`RWAIT_DISABLED`, the RX windows) has a microsecond
  timeout and bails rather than hanging. A wedged radio must never freeze the loop.
- A **hardware watchdog** (~8 s, armed in `setup()`) resets the chip if `loop()` ever stops feeding it — so a
  hang re-enumerates USB and re-inits RF on its own without a physical replug.
- USB HID report callbacks (`handleSet`/`handleGet` in `puck_hid`, `jcSet` in `mode_switch_pro`) run in **ISR
  context**. They only stage data (into bond slots, the relay queue, or an SPSC reply ring); the actual
  `sendReport` happens later from `task()`/`loop()` where it's safe.
- `volatile` marks state shared across the ISR/loop boundary (the relay queue, the Switch-Pro reply ring
  indices, the rumble level).

## Persistence

Two files in the nRF52 internal LittleFS:

- `cfg.bin` (`config.cpp`) — USB mode, tunables, chord assignments, persistence policy. A magic byte versions
  the layout; a mismatch falls back to clean defaults.
- `bonds.bin` (`bonds.cpp`) — the four bond records.

Mode switches (chord, WebUSB, or CDC) call `saveMode()` then reboot so the next boot enumerates the right
interface set. By default every cold boot returns to Steam mode unless "persist last mode" is enabled.

## Config & debug surfaces

- **WebUSB** (`webusb_config.cpp`): present in every mode; the browser panel reads a status blob and sets
  one-byte tunable fields or requests a mode switch.
- **CDC console** (`serial_console.cpp`): present only in puck modes. Single-letter commands toggle RF diag
  modes, poke radio registers live, switch USB mode, edit tunables, inject test haptics, and dump capture
  history. It is an `else-if` chain, so the **first** matching letter wins — a couple of letters (`C`, `H`)
  appear twice and the second occurrence is currently unreachable (preserved verbatim from the original; worth
  cleaning up if those commands are ever needed).

## RF reverse-engineering tooling

`rf_diag.cpp` holds everything used to *discover* the protocol — promiscuous capture, config sweeps, bit-exact
replay, the session sniffer, scan-then-respond. The protocol is solved, so none of it runs in normal use, but
it's kept compiled and reachable (via the CDC console) because it's the only way to re-derive parameters if a
controller firmware update ever changes them. It depends only on `radio` and `bonds`, so it stays out of the
operational path in `rf_link`.
