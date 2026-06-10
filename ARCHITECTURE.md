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
| `puck_hid.{h,cpp}` | **Steam/Lizard** personality: the puck HID slot interfaces, the feature command channel (bond read/write, attributes), the seamless auto-lizard decision, and the USB connection presentation. |
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
| `wake_hid.{h,cpp}` | A boot-mouse HID interface added to the clean controller modes so the host honors USB remote-wakeup (see "Wake from sleep"). |
| `status_led.{h,cpp}` | Wake-sent LED indicator: dark in all steady states; flashes 500 ms at each `remoteWakeup()`. Drives both the Feather user LED (P1.15) and the SuperMini clone's blue LED (P0.15); pins/polarity overridable. |

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

- **PHY/framing** (decoded from real-puck capture): `Ble_2Mbit`, big-endian, whitening off, 5-byte address,
  CRC16 `0x11021`/`0xFFFF` (address included). On-air address is the bit-reversed stored bytes.
- **Two addresses — discovery vs session**: the *discovery/rendezvous* address is the shared, controller-known
  `"ibex"` on channel 2; every puck uses it so any controller can find any puck. The *session* address is
  **unique per device** (`g_sessBase`/`g_sessPrefix`, derived from the FICR DEVICEID in `rfGenSessionAddr()`).
  This mirrors the real puck and is what keeps two OpenPucks from colliding on-air (see "Multi-puck isolation").
- **Bonding/reconnect**: we transmit a *host frame* — on the shared `"ibex"` rendezvous (ch2) for a searching
  controller, and on our unique session address as a keepalive once connected. Both carry the bonded
  controller's UUIDs plus the **session** base/prefix/channel we want it to adopt. The controller filters by
  UUID, adopts the advertised session address, and the connected poll then runs there (default channel 18).
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

## Multi-puck isolation

Because the discovery address is shared, two things prevent one puck from disturbing another:

1. **Unique session address.** After bonding, each puck moves the controller onto its own per-device session
   base/prefix (advertised in the host frame). The connected poll and its RX windows run there, so a puck only
   ever hears its own controller — not a neighbor's session traffic.
2. **Reply-type gate.** The link-alive timestamp (`g_connReplyMs`, which drives the "controller reconnected →
   wake the host" remote-wakeup) is stamped only on genuine controller replies (F-type: `0xF1`/`0xF2`/`0xF3`),
   never on a puck→controller frame (E-type: `0xE1` host frame, `0xE2/E3/E7`). This is belt-and-suspenders: it
   means even a neighbor's `0xE1` discovery beacon picked up on the shared rendezvous can't be mistaken for a
   reconnect. (Before these two fixes, plugging a second puck into another computer would wake a sleeping host
   running the first puck.)

## Wake from sleep

Waking the host is an **explicit gesture, never a side effect of input**. While the bus is suspended, the puck
personality sends *nothing* (no `0x45` forwards, no lizard mouse/keyboard, no periodic `0x79`/`0x7B` status) —
a suspended-bus `sendReport` can itself translate into a host wake, which made the PC wake on any trackpad
graze and nearly impossible to keep asleep. The only wake triggers are a **Steam-button short press** or a
**controller connect**, detected in `rf_link.cpp` (guarded by `USBDevice.suspended()`). Each fires two things:

1. `USBDevice.remoteWakeup()` — the device-level USB resume signal; and
2. `g_active->wakeEvent()` — queues a **wake nudge**, because a bare resume signal is **not** enough to wake
   some hosts (Windows especially): they only wake when real mouse/keyboard input *follows* the resume. Once
   the bus is back up, the puck personality plays a **mouse jiggle** on its own report (`wakeNudgeTask` in
   `puck_hid.cpp`): move a few px right, then back — net-zero cursor, no button. Real activity wakes the host
   but clicks/activates nothing (an open Start menu stays open). It can't be sent *during* suspend, so it's
   delivered right after resume. (Earlier versions sent a click / space / Ctrl too; the click + space were
   activating the focused Start tile and launching the browser, so they were dropped.)

Separately, `onReport45()` **mutes input forwarding for ~1.5 s after resume** (`POST_RESUME_MUTE_MS`): right
after wake the controller's own input — a trackpad click/trigger, or residual button state from the wake
gesture — would otherwise be forwarded as a real click/keypress into the just-woken desktop, which was what
activated the highlighted Start tile (Edge) on every wake. The jiggle is exempt (it's sent directly, not via
the forwarding path).

The board LED is a wake debugger (`status_led.cpp`): dark in all steady states (including while armed), and a
500 ms flash at the moment a `remoteWakeup()` is actually sent. Flash + host stays asleep = the resume was sent
and the host ignored it (fix host-side, e.g. `powercfg /deviceenablewake`); no flash = the firmware never fired
(it didn't see the gesture, or didn't consider the bus suspended). USB remote wakeup is a **device-level**
signal — one wake line per device, armed by a single `SET_FEATURE(DEVICE_REMOTE_WAKEUP)`; there is no
per-interface arming on the wire. Which Windows device *node* gets credited is host-internal policy.

Every mode advertises the remote-wakeup capability in its config descriptor, so the device is always *armed*.
The catch is that a host only *honors* a resume from an allow-listed input device class (HID keyboard/mouse) —
a bare gamepad/vendor/composite presentation gets armed but ignored (notably Windows under Modern Standby).
That's why Xbox mode (which exposes a boot mouse) woke Windows while the gamepad/puck modes didn't.

Why a function (not just the device) must be wake-capable: device-level remote-wakeup is necessary but not
sufficient on Windows — a *function driver* must hold a pending wait-wake request (`IRP_MN_WAIT_WAKE`) for the
system to actually wake, and only the mouse/keyboard class drivers (`mouhid`/`kbdhid`) arm one by default
(that's the "Allow this device to wake the computer" checkbox). The puck's generic-HID / CDC / WebUSB functions
don't, so its armed device-level wake was getting ignored. A boot-mouse function fixes that.

The fix: `wake_hid.cpp` registers a do-nothing **boot mouse** interface (the same shape Xbox mode already uses
to wake Windows) so each mode exposes a `mouhid`-bound, wait-wake-armed function. It's added in `setup()` for
every **clean** mode and for **puck mode on a normal boot**.

**Puck mode and the endpoint budget.** The nRF52840 has only 7 data IN endpoints. The puck composite used all
7 (CDC = 2, 4 puck HID, WebUSB = 1), leaving no room for a wake interface. So puck mode now **drops the CDC
serial console by default** (6 IN: 4 HID + WebUSB + wake mouse) — gaining Windows wake at the cost of the
serial debug console. When you genuinely need the console, arm the **one-shot debug CDC** (WebUSB panel field
20, or the CDC `D` command): the *next* boot keeps CDC and skips the wake mouse, and the boot after that reverts
automatically (`g_debugCdcThisBoot`, stored in the reused `Cfg.rsvd0` byte, consumed in `loadCfg` exactly like
`bootMode`). The two variants use different `bcdDevice` (0x0211 normal / 0x0212 debug) so Windows re-reads when
you switch. (Trade-off to verify on hardware: the code historically warned that clearing the puck's CDC
composite hurt simultaneous Steam + Chrome-WebUSB on Windows; the `bcdDevice` bump + MS OS 2.0 descriptors
should keep WebUSB binding, but confirm Steam pairing + the browser panel still work.)

Note hosts differ in what they honor: Windows keys on the input device class (this fix targets it); Linux/Steam
Deck gates wake per-device in `/sys/.../power/wakeup` regardless of class, so a mode that still won't wake the
Deck may just need that sysfs flag enabled host-side.

**`bcdDevice` rule:** Windows caches a device's configuration descriptor keyed by `VID:PID:bcdDevice`. If you
change a mode's interfaces (as adding the wake mouse did) without bumping that mode's `setDeviceVersion(...)`,
Windows keeps serving the stale cached descriptor — the change is invisible and wake silently breaks, with no
fix short of the user manually uninstalling the device. So: **whenever you change a mode's USB descriptor, bump
its `bcdDevice`.** The clean modes were bumped for the wake interface; each `begin()` documents its value.

## RF reverse-engineering tooling

`rf_diag.cpp` holds everything used to *discover* the protocol — promiscuous capture, config sweeps, bit-exact
replay, the session sniffer, scan-then-respond. The protocol is solved, so none of it runs in normal use, but
it's kept compiled and reachable (via the CDC console) because it's the only way to re-derive parameters if a
controller firmware update ever changes them. It depends only on `radio` and `bonds`, so it stays out of the
operational path in `rf_link`.
