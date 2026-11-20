# PS5 console mode (mode 13) — auth adapter

A PS5 only accepts third-party wired controllers that pass Sony's
challenge/response check, and it ignores input reports that aren't signed.
Firmware can't do either of these on its own. Mode 13 borrows the signing from a
"P5General"-class auth adapter (tested: **P5 NEX Plus**, USB `2B81:0101`,
manufacturer string `LiteEMF`). It uses the same scheme as GP2040-CE's
P5General mode:

- OpenPuck tells the PS5 it is the adapter: same VID/PID and HID descriptor.
- The console's auth feature reports go to the adapter, and the answers come
  back to the console:
  - F0 = challenge in
  - F2 = signing status out
  - F1 = signature pages out
- Every input report goes to the adapter as an interrupt-OUT report. The adapter
  sends it back with a sequence number (bytes 12–15) and an 8-byte hash
  (bytes 56–63). That signed copy is what the PS5 receives.

## Why extra hardware

The nRF52840 has a single USB controller, and it can only act as a USB device.
That controller is already busy talking to the PS5. The adapter is also a USB
device, so it needs a USB **host** to plug into. A MAX3421E USB-host chip on SPI
provides one. The firmware polls the chip from `loop()`.

## Parts

- Pro Micro nRF52840 (nice!nano clone): the normal OpenPuck board.
- A MAX3421E board. Either works:
  - **"USB Host Shield Mini"** (blue, 3.3 V, about $5). Needs the VBUS mod
    below.
  - **Adafruit MAX3421E breakout** (#5858). It has its own 5 V-in for VBUS.
- The P5 auth adapter, plugged into the MAX3421E board's USB-A port.

## Wiring

| MAX3421E | Pro Micro pin | nRF GPIO |
|---|---|---|
| SCK | D2 | P0.17 |
| MOSI | D3 | P0.20 |
| MISO | D4 | P0.22 |
| SS / CS | D5 | P0.24 |
| INT | leave unconnected (the chip is polled) | — |
| VCC / 3V3 | VCC (3.3 V) | — |
| GND | GND | — |
| RST / MAX_RST | VCC (3.3 V). **Required** after the VBUS mod: on "UHS mini v2.0" clones the on-board 2K2 reset pull-up hangs off the VBUS/3.3 V trace you cut | — |
| **VBUS (USB-A 5 V)** | **5 V from the puck's USB input**, see below | — |

On a Pro Micro nRF52840 (nice!nano or SuperMini clone), the four SPI pins are
the four pads in a row directly under the two GND pads on the left edge (USB
end up, components facing you). SuperMini silkscreen numbers them by GPIO:
`017 020 022 024`.

```
                  USB-C (to PS5)
               ┌───────────────┐
         006   │ ●           ● │ B+   battery — NOT 5 V, leave alone
         008   │ ●           ● │ GND
         GND   │ ●           ● │ RST
 shield GND ── │ ● GND   VCC ● │ ── shield 3V3  and  shield RST
 shield SCK ── │ ● 017       ● │ 031
 shield MOSI── │ ● 020       ● │ 029
 shield MISO── │ ● 022       ● │ 002
 shield SS  ── │ ● 024       ● │ 115
         100   │ ●           ● │ 113
         011   │ ●  (INT n/c)● │ 111
         104   │ ●           ● │ 010
         106   │ ●           ● │ 009
               └───────────────┘
 USB-C VBUS (5 V, tapped at the connector/diode) ── shield VBUS pad
```

To use different pins, override `P5_PIN_SCK/MOSI/MISO/CS` at build time, e.g.
`make build EXTRA_FLAGS="-DP5_PIN_CS=2"`. The values are Feather-variant
Arduino pin numbers, as listed in `mode_ps5_auth.cpp`.

**5 V for the adapter.** The adapter is a USB device and needs ~5 V on VBUS.

- Take 5 V from the puck's own USB-C VBUS, which is the PS5's 5 V. Use the VBUS
  pad or diode next to the connector.
- Check it with a meter while the puck is plugged in, before you connect it. On
  these clones, B+/RAW is the *battery* pin. Don't use it unless the meter shows
  ~5 V there.

**USB Host Shield Mini VBUS mod.** This board ties USB-A VBUS to its 3.3 V
rail. Cut the jack's VBUS trace off the 3.3 V net (two cuts on the "UHS mini
v2.0" clone), then feed the jack 5 V on its own. If you skip this, the adapter
gets 3.3 V, and 5 V reaches your 3.3 V rail once you add the feed. The
[PS5 console mode hardware](https://github.com/safijari/openpuck/wiki/PS5-console-mode-hardware)
wiki page has the step-by-step version with photos.

## Bring-up

1. Flash the firmware and select **mode 13** ("PS5 console (auth adapter)") in
   the WebUSB panel. Like the other console modes, it drops WebUSB. To get back
   to the panel, use back-4 + A (Steam mode).
2. LED: an **even ~1 Hz blink** means the MAX3421E's oscillator isn't running
   (chip held in reset: wire MAX_RST to 3.3 V, or no chip on SPI). A **slow blink** means the adapter isn't enumerating behind the
   MAX3421E (check wiring and 5 V). **Solid** means the adapter is up.
3. Bench check without a PS5: plug the puck into a computer and run
   `uv run --no-project --with hid tools/p5_check.py`. It should report a
   signed input stream at ~250 reports/s with changing hashes. Run against the
   bare adapter, it checks that the adapter signs at all.
4. Plug into the PS5. The console runs auth within a few seconds of enumeration.

## Status / limits

- **Tested on a real PS5** with a modded "UHS mini v2.0" MAX3421E board and a
  P5 NEX Plus: auth passes, and buttons, sticks and motion work.
- Motion follows the DualSense report: little-endian gyro/accel at bytes 16-27
  (GP2040-CE byte-swaps these, which sends motion haywire). Bytes 28-29 are a
  16-bit IMU timestamp the console integrates the gyro over. Left at zero,
  rotation is dead. Its tick unit is still being tuned. Byte 30 is the
  battery/status byte (`0x1A` = charging, full). Anything else there shows up
  as "battery low".
- The controller-definition report (feature `0x03`) advertises vibration
  (capability `0xCF`; the adapter's own `0xC3` leaves it off).
- **No rumble on PS5 yet.** Output report `0x02` is forwarded with the
  DualSense motor layout (byte 2 = high-frequency, byte 3 = low-frequency), and
  this works from a PC. But in Astro's Playroom the console sent only
  lightbar, LED and volume updates. PS5-native games drive DualSense haptics
  as audio, which a non-DualSense device never receives. PS4 titles are
  untested.
- Diagnostics: GET feature `0xEE` reports MAX3421E/adapter state, and
  `0xD0`-`0xD8` return the console output reports captured during the last
  PS5 session (saved to flash only on boots where a console sent an auth
  challenge).
- Only `2B81:0101` adapters are recognised.
