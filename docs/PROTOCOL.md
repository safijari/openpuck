# OpenPuck Controller Protocol

This document describes the complete controller-to-puck protocol implemented by `OpenPuck.ino`.

## 1. Scope

OpenPuck emulates the Valve Steam Controller 2 "puck" dongle. There are two separate protocols:

1. USB host-facing protocol between the puck and the computer.
2. 2.4 GHz radio protocol between the puck and the controller.

Both are required for a fully compatible implementation.

## 2. Bond record

A bond slot is 24 bytes:

```text
offset  size  meaning
0x00    4     proteus_uuid / token, little-endian
0x04    4     ibex_uuid, little-endian
0x08    16    controller serial, ASCII, not null-terminated if fully filled
```

The puck stores four independent bond slots. Slot `N` is exposed as USB HID interface `N`.

An empty slot is all zeroes.

## 3. USB protocol

### 3.1 Steam-mode USB layout

Steam mode enumerates as Valve `28DE:1142` and exposes four HID interfaces with the same report descriptor as the real puck. Each interface owns one bond slot. The firmware uses the classic Steam Controller dongle PID because SDL's Steam HIDAPI driver reliably treats `0x1142` as the four-slot wireless receiver; newer puck PIDs may work in Steam but are not consistently surfaced as Steam Controllers to SDL apps. SDL also expects the Steam slot HIDs on USB interface numbers 1..4, so WebUSB is registered before the slot HIDs in puck mode to reserve interface 0.

Relevant report IDs:

- Input: `0x40`, `0x41`, `0x42`, `0x43`, `0x44`, `0x45`, `0x79`, `0x7B`
- Output: `0x80` to `0x89`
- Feature: `0x01`, `0x02`

### 3.2 Feature commands

Feature payload framing is:

```text
[cmd][len][payload...]
```


Implemented commands:

- `0x83`: `GET_ATTRIBUTES_VALUES`
- `0xAE`: `GET_STRING_ATTRIBUTE`
- `0xB4`: `DONGLE_GET_WIRELESS_STATE` (connection state for the interface's slot)
- `0xAD`: `ENABLE_PAIRING` (pairing mode enable/disable)
- `0xA2`: `TRITON_A2_OBSERVED_PAIRING_RECORD` (write or clear the interface's 24-byte bond slot)
- `0xA3`: `TRITON_A3_BOND_EVENT_OR_STATUS` (read the interface's 24-byte bond slot)

For a full list of commands the controller supports, check steam_commands.h

#### `0xB4` response

```text
[0xB4][0x01][state]
```

- `0x02`: controller connected on this slot
- `0x01`: not connected

#### `0xA2` write

`len` must be at least `24`. A zero-filled 24-byte payload clears the slot. Any non-zero valid payload writes the bond verbatim.

#### `0xA3` read

Returns:

```text
[0xA3][0x18][24-byte bond]
```

### 3.3 Host-to-controller relay

When Steam writes feature report `0x01`, OpenPuck forwards it over RF to the controller as an `E3` frame
containing a sub-TLV. There are two on-air forms:

```text
haptic   [E3][1+len][0x05][report_id][data...]             (type 05, no inner-len)
normal  [E3][2+len][0x01][report_id][innerlen][data...]    (type 01, KEEPS inner-len)
```


```text
haptic on    E3 04 05 82 01 01 F7              (report 0x82, legacy)
brightness   E3 05 01 87 03 2D <val> 00        (report 0x87 reg 0x2D, LANDING)
power-off    E3 06 01 9F 04 6F 66 66 21        (report 0x9F "off!", LANDING)
```

The relay carries the command's declared length, up to 60 bytes for type 01 commands and 63 for type 05 haptic
reports (a full OUTPUT `0x87`/`0x88` sample frame). Relays are staged in a small ring (not a single buffer): the
USB SET callbacks run in ISR context and Steam sends settings/calibration as back-to-back bursts, so a single
pending slot both drops reports and can be torn mid-flush. One queued relay is emitted per poll cycle, plus a
second in the same cycle while the ring still holds a backlog (PCM sample streams need ~258 frames/s).

### 3.4 Connection presentation (input reports `0x79` / `0x7B`)

- `0x79` (1 byte): connection state, `0x02` = connected, `0x01` = disconnected. **Edge-triggered**, like the
  real puck. OpenPuck additionally re-sends it every 750 ms after a connect edge *until Steam reacts* (its
  first OUTPUT/settings write after the edge), covering a missed edge. It must not be re-sent unconditionally:
  each `0x79=02` can re-trigger Steam's connect handling (connect chime) and loop the haptic before Steam
  starts consuming `0x45`.
- `0x7B` (12 bytes): periodic status, sent every 2 s while connected. Byte 8 is the controller→puck signal
  strength as **signed dBm** (live-captured example `0xDD` = −35 dBm). OpenPuck fills it with the smoothed
  radio RSSI sampled on each CRC-good controller reply (`RSSISAMPLE`, started by an `ADDRESS→RSSISTART`
  short during the poll RX window). The other bytes are replayed from the live capture
  (`F7 01 89 00 00 00 03 00 __ 00 3A 02`); their semantics are unconfirmed.

## 4. Radio physical layer

OpenPuck uses Nordic raw radio mode configured to match the puck firmware:

- PHY: `Ble_2Mbit`
- Dynamic-length packet format
- `PCNF0 = 0x00030008`
- `PCNF1 = 0x01040040`
- 5-byte address (`BALEN=4`)
- Big-endian
- Whitening disabled
- CRC16, address included
- Polynomial `0x11021`
- Init `0xFFFF`

CRC is standard CRC-16-CCITT over address plus payload, MSB-first.

### 4.1 Address transform

Stored address bytes are converted to on-air address bytes by bit-reversing each byte.

For the nRF `BASE0` register, the bit-reversed base bytes are then packed big-endian:

```text
BASE0 = bitrev8(base[0]) << 24 |
        bitrev8(base[1]) << 16 |
        bitrev8(base[2]) <<  8 |
        bitrev8(base[3])

PREFIX0 = bitrev8(prefix)
```

## 5. Discovery / reconnect

Discovery uses a fixed address and fixed channel:

- Base bytes: ASCII `"ibex"` = `69 62 65 78`
- Prefix: `0x10`
- Channel: `2`

The puck transmits a host frame on that address until the controller connects.

## 6. Host frame (`E1`)

OpenPuck's discovery host frame is a dynamic-length ESB payload:

```text
RAM buffer:
[0]  = 0x12                     ; payload length
[1]  = S1 byte                  ; PID in bits 1..2, no-ack bit clear
[2]  = 0xE1                     ; host-frame opcode
[3..6]   = proteus_uuid         ; 4 bytes, little-endian
[7..10]  = ibex_uuid            ; 4 bytes, little-endian
[11]     = session channel
[12..14] = 0x00 0x00 0x00
[15..18] = session base         ; 4 bytes, stored form
[19]     = session prefix
```

Semantics:

- `proteus_uuid` is the first 4 bytes of the bond record.
- `ibex_uuid` is the next 4 bytes of the bond record.
- The host chooses the connected-session channel and address and advertises them here.
- The session base/prefix are PER-BOND (one address per controller, derived from the bond UUID + the
  puck's FICR DEVICEID so two pucks that bond the same controller still isolate). The real puck
  supports up to four simultaneous controllers on the same channel; OpenPuck mirrors that.

## 7. Connected session

After discovery, the puck becomes the poll master.

### 7.1 Controller wake / version frame (`E7`)

OpenPuck sends:

```text
E7 00 00
```

This is the normal "host awake" form.

### 7.2 Input poll (`E3`)

OpenPuck polls for report `0x45` using:

```text
E3 [len] 0x01 0x45 [param]
```

- `0x01`: GET subtype
- `0x45`: controller input report
- `param`: implementation tunable, usually `0x00` or `0x2D`

The controller replies in the ACK window.

### 7.3 Controller replies

Observed reply opcodes:

- `0xF1`: full TLV container carrying input data
- `0xF2`: disconnect / shutdown
- `0xF3`: status

#### `0xF1` container

The TLV scan starts at byte 6. Each record is:

```text
[len][tag][value...]
```

Known tags:

- `0x02`: 4-byte control/status field
- `0x04`: bulk data blob
- `0x06`: embedded report wrapper

For controller input, the embedded report is `0x45`.

## 8. Report `0x45` layout

Report `0x45` is 46 bytes:

```text
offset  size  meaning
0x00    1     report id = 0x45
0x01    1     sequence
0x02    4     buttons, little-endian u32
0x06    2     left trigger, u16
0x08    2     right trigger, u16
0x0A    2     left stick X, s16
0x0C    2     left stick Y, s16
0x0E    2     right stick X, s16
0x10    2     right stick Y, s16
0x12    2     left pad X, s16
0x14    2     left pad Y, s16
0x16    2     left pad press, s16
0x18    2     right pad X, s16
0x1A    2     right pad Y, s16
0x1C    2     right pad press, s16
0x1E    4     IMU timestamp, u32
0x22    2     accel X, s16
0x24    2     accel Y, s16
0x26    2     accel Z, s16
0x28    2     gyro X, s16
0x2A    2     gyro Y, s16
0x2C    2     gyro Z, s16
```

### 8.1 Button bitfield

```text
0x00000001 A
0x00000002 B
0x00000004 X
0x00000008 Y
0x00000010 QAM
0x00000020 R3
0x00000040 View
0x00000080 R4
0x00000100 R5
0x00000200 RB
0x00000400 D-pad down
0x00000800 D-pad right
0x00001000 D-pad left
0x00002000 D-pad up
0x00004000 Menu
0x00008000 L3
0x00010000 Steam
0x00020000 L4
0x00040000 L5
0x00080000 LB
0x00100000 right stick touch
0x00200000 right pad touch
0x00400000 right pad click
0x00800000 right trigger click
0x01000000 left stick touch
0x02000000 left pad touch
0x04000000 left pad click
0x08000000 left trigger click
0x10000000 right grip touch
0x20000000 left grip touch
```

## 9. USB presentation modes

The RF side stays the same across modes. Only USB enumeration changes.

There are three switchable modes (`0=Steam 1=Xbox 2=Switch`). Steam mode is a CDC + WebUSB composite;
Xbox and Switch are **clean controller-only** devices — the auto-added CDC/WebUSB
interfaces are torn down (`clearConfiguration`) and `bcdUSB` stays `0x0200` (no USB-2.1 BOS). This is
required because Windows' `xusb` driver expects the Xbox 360 controller on the primary device/interface,
and a real Switch console rejects composite devices. Every boot/mode-switch
does a `detach -> rebuild -> attach` so the host re-reads the descriptor cleanly.

### 9.1 Steam mode

- VID:PID `28DE:1142`
- Four puck HID interfaces (CDC + WebUSB also present)
- `0x45` reports are forwarded to the connected slot's HID interface
- **Seamless lizard**: when Steam is driving the device, `0x45` is forwarded; when it isn't
  (Steam closed, 7 s watchdog) the same `0x45` is translated into mouse (`0x40`) + keyboard (`0x41`)
  reports on the **same** puck interface, so the device is a driverless desktop keyboard+mouse with no
  mode switch. This is purely USB-side; the RF poll and relay are unchanged. (There is no standalone
  lizard mode.)
- **"Steam is driving" signal**: *any* Steam OUTPUT/settings report (`0x80`–`0x89`, e.g. the `0x87`
  lizard-off heartbeat, LED, or a `0x82` haptic) refreshes the watchdog — not just `0x87`. This makes
  the puck leave lizard for gamepad on Steam's **first** contact, even if that first packet is a haptic
  that arrives before the heartbeat.
- **Haptics are gated on this decision**: haptic reports are **never** relayed to the controller while
  the puck is presenting lizard. Relaying haptics while Steam isn't reading `0x45` back made Steam loop
  the same command, leaving the controller buzzing; suppressing them keeps the lizard state clean. The
  same gate applies during the post-resume input mute (`POST_RESUME_MUTE_MS`): Steam can't read `0x45`
  back in that window either, so a wake-time haptic would loop identically.
- **Which OUTPUT reports are relayed**: all haptic/actuator reports `0x80`–`0x89` are forwarded to the
  **connected slot only** (the slot gate is what stops a haptic aimed at another of the four exposed
  slots from buzzing the single controller). The 63-byte `0x87`/`0x88`/`0x89` are raw haptic sample
  streams (table below), used by audio-to-haptics apps; the real puck forwards every OUTPUT report
  unfiltered. (The feature-`0x01` command `0x87`, `SET_SETTINGS_VALUES`, is a different id space and
  reaches the controller via the feature passthrough.) Steam owns `IMU_MODE` in native puck mode. Switch Pro mode instead writes
  raw accelerometer + gyroscope (`0x18`) when the RF link comes up because the controller retains Steam's
  IMU-off state across pucks. Restricting haptics to `0x82` alone silently dropped the ping/grip/test
  haptics, which use other report IDs.

#### ⚠️ Two different `0x8x` id spaces — never share a rule between them

Steam drives the actuators through the Triton **OUTPUT report** space, which is a *different* id space
from the feature-`0x01` **command** space even though the numbers overlap. Ground truth: SDL
`src/joystick/hidapi/steam/controller_structs.h` (`ValveTritonOutReportMessageIDs`) and
`controller_constants.h`. The payload sizes match this firmware's report descriptor exactly:

| id | OUTPUT report (haptics/actuators) | payload | feature `0x01` command |
|----|-----------------------------------|---------|------------------------|
| `0x80` | `HAPTIC_RUMBLE` `[type][intensity u16][{speed u16,gain}L][{…}R]` | 9 | `SET_DIGITAL_MAPPINGS` |
| `0x81` | `HAPTIC_PULSE` `[side][on_us u16][off_us u16][repeat u16]` | 7 | `CLEAR_DIGITAL_MAPPINGS` |
| `0x82` | `HAPTIC_COMMAND` `[side][command][gain_db]` | 3 | `GET_DIGITAL_MAPPINGS` |
| `0x83` | `HAPTIC_LFO_TONE` `[side][gain_db][freq u16][dur u16][lfo_freq u16][lfo_depth]` | 9 | `GET_ATTRIBUTES_VALUES` |
| `0x84` | `HAPTIC_LOG_SWEEP` `[side][gain_db][dur u16][start u16][end u16]` | 8 | `GET_ATTRIBUTE_LABEL` |
| `0x85` | `HAPTIC_SCRIPT` `[side][script_id][gain_db]` | 3 | `SET_DEFAULT_DIGITAL_MAPPINGS` |
| `0x86` | PCM mode `[op][channel][format]`: op 2 enables with format (0-3 16-bit, 4-7 8-bit, 8-11 u-law, each 8/4/2/1 kHz), op 1 sends the stream a stop message; channel 1 right grip, 2 both grips, 3 left touchpad, 4 right touchpad, 5 both touchpads | 3 | `FACTORY_RESET` |
| `0x87` | mono sample stream `[target][samples]`: target 0 left grip, 2 or `0x80` both grips, 3 left touchpad, 4 right grip, 5 both touchpads (1 and the right touchpad alone are not addressable) | 63 | `SET_SETTINGS_VALUES` |
| `0x88` | stereo **grip** stream `[n<=31][31 samples left grip][31 samples right grip]` in the `0x86` format | 63 | `CLEAR_SETTINGS_VALUES` |
| `0x89` | length-prefixed `0x87`: `[len][0x87 payload]` | 63 | `GET_SETTINGS_VALUES` |

Actuator routing: the controller has four LRAs, one under each trackpad and a higher-output one in each grip.
- `0x80` rumble plays on **both grips**.
- `0x82` / `0x83` side (bit 7 ignored): 0 left touchpad, 1 right touchpad, 2 both touchpads, 3 left grip,
  4 right grip, 5 both grips. `0x81` is the same except 0 = right touchpad and 1 = left touchpad.
- `0x88` PCM plays on the **grips** (first half left, second half right); `0x87` / `0x89` as in the table.

PCM streaming, measured with the controller's IMU over USB (2026-10-03):
- `0x86` sets the sample format, and the format persists. Op 1 does not stop playback, and channel 5 measured the
  same as channel 2. OpenPuck sends `{2, 2, 9}` (4 kHz u-law) before each stream and every second while streaming.
- The controller pre-buffers about 40 ms (onset ~43 ms against ~16 ms for a `0x83` tone). It rides out 124 ms bursts
  and 0-20 ms jitter without a dip, and falls silent by itself 60-85 ms later than a tone once frames stop
  (op 1 at the stop trims that to about 50 ms). The playback clock does not drift against 4 kHz pacing. `0x82`
  does not cut a stream.
- A stream starts on the first frame to arrive once more than 16 ms of samples are queued, and that fill stays
  queued for the whole stream; running dry stops it. With 31-sample 4 kHz frames, playback starts at 124 samples
  (31 ms). OpenPuck makes the third frame of each stream 3 samples (`n` = 3, same 63-byte layout) so it starts at
  96 (24 ms). Measured through OpenPuck in Steam mode (20 runs each, 2026-10-04): IMU onset median 61 ms with full
  frames, 54 ms with the short third frame.
- Level: a u-law sample amplitude of about 0.4 (0.31 at 100 Hz, 0.49 at 320 Hz) matches a -3 dB `0x83` tone.
- An 8 kHz stereo stream is 258 frames/s, above one relay per 4 ms poll. OpenPuck flushes a second queued relay in
  the same cycle when the ring holds a backlog, and resends an unanswered `0x86`-`0x89` frame once with the same PID.

OpenPuck uses this stream itself for the DualSense wave haptics style (DUALSENSE_HAPTICS.md) and for Switch Pro HD
rumble (§9.5).


### 9.2 Xbox mode

- VID:PID `045E:028E`, clean device (no CDC/WebUSB)
- Custom XInput-compatible vendor interface on `MI_00` + HID boot mouse on `MI_01` (right-pad emulation)
- `0x45` is converted into a 20-byte XInput report

### 9.3 Switch mode

- VID:PID `0F0D:0092` (HORI Pokkén Tournament Pro Pad), clean device (no CDC/WebUSB)
- Single HID interface with the canonical HORIPAD descriptor (interrupt IN + OUT endpoints), accepted by
  a real Switch console with no handshake; an 8-byte report is streamed at ~250 Hz

### 9.4 Original Xbox mode

- VID:PID `045E:0289` (Microsoft Controller S). Composite: the XID interface plus the wake mouse and
  the WebUSB panel, so the config panel stays reachable on a PC
- Proprietary XID interface (class `0x58`, subclass `0x42`, protocol `0x00`) with interrupt IN +
  OUT endpoints; 20-byte input reports, rumble applied from the OUT endpoint
- The console refuses the controller until three `0xC1` vendor control requests are answered:
  `bRequest 0x06` / `wValue 0x4200` (XID descriptor), and `bRequest 0x01` with `wValue 0x0100` /
  `0x0200` (input and output capabilities)
- Also answers the XID report requests the protocol carries on EP0, in parallel with the interrupt
  endpoints: `GET_REPORT` (`0xA1 0x01`, `wValue 0x0100`) and `SET_REPORT` (`0x21 0x09`, `wValue 0x0200`)

### 9.5 Switch Pro HD rumble

The console streams HD rumble in every output report: per side, a 4-byte word packing one or more amplitude and
frequency updates for a low band (around 160 Hz) and a high band (around 320 Hz), in 1/32 log2 units
(`hdrDecode` in `mode_switch_pro.cpp`). Amplitude code 100 is full scale, as in the public amplitude table and
SDL's encoder; codes above it play at full scale. OpenPuck renders the latest bands in the main loop
(`hapticHdTask` in `haptics.cpp`), not per USB packet:

- **Grips:** each side's low and high band, summed and phase-continuous, as a 4 kHz `0x88` PCM stream on that
  side's grip, where a Pro Controller has its actuators. Scaled by the rumble strength (field `22`) and rounded off
  by the grip limiter (field `115`). The stream stays up through 500 ms of silence so a new effect doesn't pay the
  controller's pre-buffer again, and ends with `0x86` op 1.
- **Trackpads:** each side's high band as an `0x83` tone on that side's trackpad, scaled by the HD trackpad
  strength (field `141`) and held at or below -15 dB between 250 and 300 Hz, where the pads ring loudly. Tones
  last 120 ms and are refreshed every 50 ms, re-sent at once on a +6 dB hit and at most every 32 ms on a retune;
  a band going quiet is cut with -128 dB.
- The output stops when no rumble arrives for 600 ms, the per-type rumble toggle is off, the host suspends, or the
  mode changes. Switch Pro always renders HD rumble; the rumble style (field `39`) applies to the other modes.

## 10. WebUSB control channel

The WebUSB vendor interface is present only in Steam mode (Xbox/Switch are clean controllers with no
config interface — configure them from Steam mode or switch via the back-paddle chord).

Messages:

- Host to device:
  - `0x01`: get status blob
  - `0x02 <field> <value>`: set one field. Notable fields: `22` host-rumble strength as **percent/2**
    (10–500%, revived in blob version 21), `39` host-rumble style (`RUMBLE_STYLE_*` in `haptics.h`:
    0 normal, 1 mono, 2 heavy, 3 light, 4 swapped, 5 punchy, 6 soft; not used in Switch Pro mode, which
    renders HD rumble, §9.5), `38` Switch Pro gyro mapping.
    DualSense audio haptics: `31` on/off, `30` gain as percent/2 (10-500%, 0 = automatic), `88` style
    (0 rumble, 1 tone, 2 split, 3 wave; default 3). See DUALSENSE_HAPTICS.md.
    Grip limiter (blob version ≥ 22): `115` soft-limit knee of the grip PCM stream, percent of full scale,
    50..100 (70 default, 100 = off: hard clip only); other values are ignored. One setting for the DualSense
    wave style and Switch Pro HD rumble grips.
    `141` (blob version ≥ 22): Switch Pro HD rumble trackpad strength as percent/2 (0-500%, 100 default;
    0 = grips only).
  - `0x03 <mode>`: switch mode and reboot
  - `0x07`: re-init haptics (clear a stuck buzz)
  - `0x08`: send controller power-off
  - `0x16`: test rumble — buzz every linked controller with the configured style/strength for 500 ms,
    auto-stopped by the firmware. **Requires status-blob version ≥ 21**; older firmware drops it silently
    (the parser only accepts `0x01`–`0x15` and `0x20`–`0x25`).
  - `0x09`: export all bond slots (reply: `0xA7` frame) — see §10.1
  - `0x0A 0x45 0x52 0x53`: factory erase (`"ERS"` magic), then reboot
  - `0x0B` / `0x0C`: reboot into serial DFU / UF2 bootloader
  - `0x0D <slot> <used> <24-byte rec>`: write one bond slot into RAM — see §10.1
  - `0x0E <mode>`: commit imported bonds (+ apply mode, `0xFF` = unchanged), then reboot
  - `0x11`: get the lizard binding map (device replies `0xAA`). **Requires status-blob version ≥ 16** —
    older firmware drops the unknown op silently and never replies, so a host that issues a blocking read
    for the `0xAA` frame would hang the shared endpoint. The panel gates all `0x11`–`0x15` sends on the
    version byte (`0xA5` payload byte 0) reaching 16 first.
  - `0x13 <count>`: begin a lizard-map edit — set the binding count
  - `0x12 <idx> <16-byte binding>`: set one lizard binding (see below)
  - `0x14`: commit the edited lizard map to flash (device echoes `0xAA`)
  - `0x15`: reset the lizard map to built-in defaults (device echoes `0xAA`)
  - `0x20`–`0x24`: staged firmware update (begin/data/end/reboot/abort), acked with `0xAB` frames
  - `0x25 0x57 0x49 0x50 0x45`: **full board wipe** (`"WIPE"` magic, debug panel only). Erases the app
    region + LittleFS (settings + bonds) + bootloader-settings page and reboots app-less, so the board mounts
    as the UF2 bootloader drive on **every** boot until firmware is flashed again. Unlike `0x0A` (factory
    reset, which keeps the firmware), this leaves no trace of OpenPuck. Irreversible without re-flashing.
- Device to host:
  - `0xA5 <len> <payload>`: status blob
  - `0xA7 <len> <payload>`: bond export (§10.1)
  - `0xA8 ...`: flight-recorder stream
  - `0xA9 ...`: live wedge report
  - `0xAA <count> <count×16-byte bindings>`: lizard binding map
  - `0xAB 5 <status> <nextOff u32 LE>`: firmware-update ack

Lizard binding wire format (16 bytes), matching `LizardBinding` in `lizard_map.h`:

```text
[0]     outType   (0 none, 1 keyboard chord, 2 mouse btn, 3 mouse axis, 4 scroll, 5 consumer)
[1..7]  outData[0..6]  (type-specific payload)
[8..11] trigMask  u32 LE  (button bits: any-of)
[12..15] holdMask u32 LE  (button bits: all-of guard)
```

> Lizard-map commands and the `0xAA` reply were renumbered from `0x0D–0x11`/`0xA7` on the merges with
> `main`, which had independently claimed `0x0D–0x10`, `0xA7` (bond export) and `0xA8` (flight
> recorder).

Status blob payload:

```text
[0]  version
[1]  usb mode
[2]  xbox mouse sensitivity divisor
[3]  xbox mouse friction
[4]  steam pad smoothing
[5]  A/B + X/Y swap
[6]  back mapping 0
[7]  back mapping 1
[8]  back mapping 2
[9]  back mapping 3
[10] connected slot or 0xFF
[11] link up flag
[12] delivered reports/s low
[13] delivered reports/s high
[14] poll interval / 100
[15] new reports/s low
[16] new reports/s high
[17] E7 mode byte
[18] relay opcode
[19] relay subtype
[20] forward-new-only flag
[21] QoS auto-hop flag
[22] persist-mode flag
```

Later fields are appended (the version byte says how far the payload goes). From version 20 the tail carries
the per-emulated-type trackpad-to-stick mapping at payload bytes 187..194 — two bytes per type
(`{left pad, right pad}`), each `0` off / `1` left stick / `2` right stick. Set with
`0x02 <80 + type*2 + pad> <value>`. Fields 40..75 are the per-type config block, so the mapping starts at 80.
A mapped pad **blends** with its stick: while the pad is touched each axis reports whichever of the two
sources is deflected further from center (signed); an untouched pad contributes nothing and the physical
stick passes straight through. A mapped pad also stops reporting as a touchpad contact / mouse.

From version 21, payload byte 193 (`p[195]`) is the rumble style. From version 22, payload bytes 194..198
(`p[196..200]`) are the DualSense audio-haptic gain as percent/2 (0 = automatic), audio haptics on/off, audio
haptics style, the grip-limiter knee in percent, and the Switch Pro HD trackpad strength as percent/2.


### 10.1 Backup / clone (bond export & import)

A puck's whole portable identity is its four 24-byte bond records (`[proteus_uuid 4][ibex_uuid 4][serial 16]`,
§2). The per-bond on-air *session address* is **not** stored or transferred: each puck re-derives it from the
bond UUID mixed with its own FICR `DEVICEID`, and the bonded controller relearns it from the `E1` host beacon on
every reconnect. So copying the bond records onto a second puck is sufficient for any controller paired to the
first to connect to the second **with no re-pairing** — the two pucks just derive different (non-colliding)
session addresses for the same controller.

This makes a puck cloneable. The WebUSB panel exposes it as **Export to file** / **Import from file**.

**Export — `0x09` → `0xA7` frame:**

```text
[0]    format version (1)
[1]    used mask (bit s = slot s bonded)
[2..]  slot 0..3 records, 24 bytes each (96 bytes)        (len = 2 + 4*24 = 98)
```

**Import — `0x0D` (per slot) then `0x0E` (commit):**

```text
0x0D  <slot 0..3>  <used 0|1>  <24-byte record>      writes one slot into RAM (used=0 clears it)
0x0E  <mode>                                         persist all slots, apply mode (0xFF=unchanged), reboot
```

Bonds are written one slot at a time so no command exceeds a single USB-FS packet, and each `0x0D`/`0x02` is
acked with a status blob so the host's read-after-write keeps the OUT pipe flowing. The panel sends all four
`0x0D` slot writes plus every config field via `0x02`, then a single `0x0E` to persist and reboot — so the
reboot regenerates the session addresses from the new UUIDs and the restored puck reproduces both the pairings
and all settings. Other settings (mouse, chords, per-type button maps, Switch Pro gyro mapping) ride in the
backup file as plain `0x02` field values, not in the bond commands.

## 11. Timing notes

- Default poll interval: `4000` microseconds (250 Hz, matches SC2 report rate)
- Default RX window: `1200` microseconds
- Discovery beacon continues on channel 2 even after a connected session exists
- Connected session can be moved to a cleaner channel such as 18 or 52

## 12. Reimplementation checklist

To build a compatible puck from scratch:

1. Expose four independent bond slots over USB.
2. Implement `0x83`, `0xAE`, `0xB4`, `0xAD`, `0xA2`, `0xA3`.
3. Persist 24-byte bond records exactly.
4. Configure the radio exactly as described above.
5. Beacon `E1` host frames on `"ibex"` / prefix `0x10` / channel 2.
6. Poll with `E7` then `E3 GET 0x45`.
7. Parse `F1` and unpack report `0x45`.
8. Relay host feature writes to the controller as `E3` SET sub-TLVs.
9. Re-enumerate USB cleanly when switching Steam/Xbox/Switch modes (Xbox/Switch as clean non-composite devices).

If those pieces match, the controller-to-puck protocol is fully reimplemented.
