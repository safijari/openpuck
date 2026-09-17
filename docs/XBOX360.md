# Xbox 360 mode (experimental)

**Stock-console support is awaiting physical validation, not verified
compatibility.** The default `adafruit:nrf52:feather52840` build passed with
Adafruit nRF52 core **1.7.0**. The updated firmware has been flashed and passed
physical PC USB authentication tests; console acceptance remains unverified
after a reported failure with the earlier build. Native regressions and the PC
probe use synthetic challenges, not real-console captures.

`MODE_XBOX` (`1`) implements a wired Xbox 360 USB personality and libxsm3 retail
authentication in software on the **existing nRF52840 OpenPuck**. No donor
controller, USB host adapter, second board, or additional wiring is required.
The Steam Controller still connects by OpenPuck's RF protocol; this is not an
Xbox 360 wireless receiver. Original Xbox mode (`MODE_XBOX_OG`, `10`) is separate.

## Use and constraints

1. Pair the Steam Controller with OpenPuck in Steam mode on a PC first.
2. In the WebUSB panel, enable **Persist last mode** before selecting Xbox
  mode. It is off by default: unplugging the puck otherwise boots it back
  into Steam mode, even if Xbox mode was selected before unplugging.
3. Hold **all four back buttons (L4 + R4 + L5 + R5) + X** to enter Xbox mode
   with the default chord assignment.
4. Unplug from the PC and connect to the console. With persistence enabled,
  the puck boots into Xbox mode. Alternatively, leave persistence off and
  select Xbox mode with the controller chord after connecting to the console.
5. To configure mappings/rumble or update firmware, connect OpenPuck to a PC and
   use **back-4 + A** for Steam mode, then open the WebUSB panel. Return with
   **back-4 + X**. A is a fixed recovery chord; B/X/Y are configurable, so a saved
   reassignment may change the Xbox shortcut. Mode switching reboots the puck.

- **One USB gamepad**, even with four stored RF bonds. The first fresh report
  from a live controller selects its bond. It stays selected until bond removal
  or more than **1200 ms** without an RF reply; another controller cannot steal
  it during a brief gap. RF supplies synthetic neutral input after **300 ms** of
  silence to release held controls, without selecting an inactive bond.
- The USB layout stays present with no RF controller and does not re-enumerate
  when a controller joins, disconnects, or is replaced.
- Gamepad input (including Guide) and host rumble are implemented. Rumble goes
  only to the selected RF controller and uses the saved translated-mode tuning.
- **No right-trackpad mouse, CDC serial, WebUSB, or wake HID** in Xbox mode.
  Use Steam/Lizard for desktop mouse input; configure Xbox settings from Steam.
- **Headset/audio and chatpad/plugin functions are not emulated.** Their USB
  interfaces describe empty accessory roles; no accessory endpoints are opened
  or serviced. LED commands are parsed, but physical player-ring behavior is
  not equivalent to a genuine pad. This is not an indistinguishable replacement.

## USB and authentication implementation

Identity: Microsoft `045E:028E`, `bcdDevice=0x0114`, `bcdUSB=0x0200`, device
class/subclass/protocol **`FF/FF/FF`**. A fixed 153-byte configuration contains
four vendor interfaces:

| Interface | Class/subclass/protocol | Role |
|---|---|---|
| 0 | `FF/5D/01` | Control: 20-byte gamepad input on `0x81`, rumble/LED OUT on `0x02` |
| 1 | `FF/5D/03` | Audio, no accessory emulation |
| 2 | `FF/5D/02` | Plugin, no accessory emulation |
| 3 | `FF/FD/13` | XSM3 security via EP0 |

The security string at index 4 is **178 bytes** (header + 88 UTF-16 code units,
no trailing terminator). A linker wrapper avoids Adafruit's shorter string
buffer; the full multi-packet USB transfer still needs hardware validation.
USB serial and XSM3 identification share a stable 12-character serial derived
from the nRF52840 hardware ID, independent of RF bond count or USB resets.

Authentication handles vendor requests to interface 3 (`wIndex=3` or `0x0103`)
and device-recipient requests (`wIndex=0`):

| Request | Direction | Purpose / full payload length |
|---|---|---|
| `0x81` | IN | Identification, 29 bytes; starts a fresh session |
| `0x82` | OUT | Init challenge, exactly 34 bytes |
| `0x86` | IN | Status, 2 bytes: state 1 pending, state 2 ready |
| `0x83` | IN | Completed response, 46 bytes for init or 22 for verify |
| `0x87` | OUT | Verify challenge, exactly 22 bytes; requires a completed session |

Invalid actual transfer lengths, packet length fields, checksums, and MACs fail
closed: no successful readiness or stale response is published. Hardware RNG
or worker startup failure also fails closed. USB **RESET** invalidates the
session generation; an in-flight crypto operation may finish, but its old
result is discarded rather than reused for a new host/session.

Crypto runs in a dedicated **idle-priority FreeRTOS task with a 2048-word
(8 KiB) stack**, not on the small USB callback stack. The main loop lends it
**1 ms only while authentication is pending/running**, between RF polls and
never inside an RX window. This keeps crypto out of USB callbacks but does not
establish real-time timing on hardware.

All three linker wrappers are required: `tud_vendor_control_xfer_cb` for vendor
routing, `tud_descriptor_string_cb` for the full security string, and
`usbd_control_xfer_cb` for actual EP0 received lengths. Use `make build` or the
complete direct Arduino command in [BUILD_AND_DEPLOY.md](BUILD_AND_DEPLOY.md).

## Validation

Non-hardware checks, from the repository root:

```sh
python3 tools/test-xinput.py --check-fixtures
make check
make build
```

The native harness checks production auth, descriptor/string callbacks, input,
rumble, controller selection, and reset/failure handling against independent
OpenSSL/Python-generated synthetic exchanges. It does **not** prove physical
enumeration, actual TinyUSB transfers, linker wrapping, FreeRTOS scheduling,
hardware RNG, or stock-console compatibility. See
[../tests/xinput/README.md](../tests/xinput/README.md) for coverage and limits.

Separately, a physically connected **ProMicro nRF52840** (`045E:028E`) passed
PC PyUSB transport testing against the independent OpenSSL/Python oracle on
2026-09-17 after flashing the keepalive fix: descriptor/string checks passed,
init became ready in **64.3 ms**, **zero-data OUT `0x84` passed in 1.1 ms**,
and all **three verifies became ready in 28.3–29.0 ms**. Responses were checked
using the controller nonce recovered from the hardware reply. This confirms
the fix for the older build's reproducible OUT `0x84` stall.
Stock-console compatibility remains
**unverified**, with a user-reported console failure; these PC results do not
establish console acceptance.

For an explicit manual repeat, use
[../tools/check-xinput-usb.py](../tools/check-xinput-usb.py) with `--run`
(optionally `--bus` and `--address`, in decimal). It requires PyUSB, a libusb
backend, USB permissions, and OpenSSL; it installs nothing. It refuses ambiguous
selection or unexpected descriptors/strings before authenticating, but those
checks cannot prove device provenance: connect only the intended OpenPuck, not
a genuine controller. Authentication **invalidates any active session;
unplug/replug afterward even on failure**. It never detaches drivers, sets the
USB configuration, resets, or flashes the device. Default success requires
zero-data OUT `0x84` after init and before all three verifies.
`--skip-keepalive` is **only an old-build diagnostic**, explicitly reports the
skip, and does not validate the fix. The default script run produced the
physical PC test results above.

### Physical test checklist (not yet performed)

Record firmware revision, board/core version, console model/dashboard version,
USB port/cable, and results. Keep captures of failed as well as successful runs.

- [ ] Cold boot with the console powered off and OpenPuck already attached,
      then power on; also attach OpenPuck to an already-powered-on console.
- [ ] Repeat with the RF controller connected **before USB** and powered on
      **after USB**. Check the static gamepad remains present without RF.
- [ ] Capture enumeration: identity, all four interfaces, and the full 178-byte
      security string. Trace `0x81 → 0x82 → 0x86 → 0x83`, then `0x87` with
      subsequent status/response reads. Confirm no ready response before crypto
      completes, and repeat authentication where requested by the host.
- [ ] Exercise USB reset and unplug/replug, including during authentication.
      Move the same puck to a **different console** and confirm a fresh session
      without stale responses or a changed hardware serial.
- [ ] Test dashboard navigation and in-game buttons, D-pad, sticks, analog
      triggers, **Guide**, and **rumble**. Note player assignment separately from
      the non-equivalent physical ring; do not treat it as an accessory test.
- [ ] Test brief RF loss (neutral at 300 ms), disconnect beyond 1200 ms, reconnect,
      and takeover by a second paired controller without USB re-enumeration or
      rumble sent to the wrong controller. Run **more than 10 minutes**, including
      reconnects and repeated rumble; check for auth loss, stuck inputs or resets.
- [ ] Check desktop binding on Windows (`xusb`/XInput) and Linux (`xpad`/SDL),
      input and rumble, plus back-4+A configuration and return to Xbox. Confirm
      Xbox mode exposes no mouse, WebUSB, CDC, or wake HID.

## Licensing and redistribution

- **libxsm3: LGPL-2.1-or-later**, Copyright 2022–2023 InvoxiPlayGames, with
  oct0xor attribution retained in the vendored sources.
- **ExCrypt: BSD-3-Clause**, Copyright 2020 emoose; the full BSD notice is in
  [../OpenPuck/src/libxsm3/NOTICE.md](../OpenPuck/src/libxsm3/NOTICE.md).
- **GP2040-CE-derived USB descriptors: MIT**, Copyright 2021 Jason Skuby;
  the full permission/copyright notice is in
  [../OpenPuck/xinput_descriptors.h](../OpenPuck/xinput_descriptors.h).

Retain the copyright notices, attribution, disclaimers, and license texts in
source and binary distributions as required. Review the vendored revision and
local modifications in
[../OpenPuck/src/libxsm3/NOTICE.md](../OpenPuck/src/libxsm3/NOTICE.md) and the full
[../OpenPuck/src/libxsm3/LICENSE.txt](../OpenPuck/src/libxsm3/LICENSE.txt).
For statically linked firmware, satisfy the applicable LGPL source and
relinking obligations: provide the corresponding library source (including
modifications) and the object files or complete buildable source and build
materials needed to relink with a modified library, using a distribution method
permitted by the license. A binary plus a notice alone is not sufficient.