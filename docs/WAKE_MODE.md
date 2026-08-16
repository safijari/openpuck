# Wake Mode (MODE_WAKE): powering on a shut-down PC from the controller

Wake mode turns the puck into a bare USB boot keyboard that types the firmware
"smart power on" hotkey (Lenovo: **Alt+P**) the moment your controller
connects, powering on a **fully shut down (S5)** machine — then reboots itself
back into the normal puck so the booted OS sees the controller as usual. USB
remote wakeup only covers sleep (S3); this covers real power-on.

**Nothing about this mode is active by default.** If you never enter mode 10,
the puck behaves exactly as before on every machine, Lenovo or not.

## 1. Requirements

- A PC whose firmware can power on from a USB keyboard hotkey. Developed and
  validated on a **Lenovo ThinkCentre M90q Gen 6** ("Smart Power On", Alt+P).
  Other vendors have equivalents under different names and different key
  chords — see §6.
- The puck plugged into the machine's **designated always-on USB port** (on
  ThinkCentres the Smart Power On port; the EC only services that port in S5).
- BIOS setting enabled: **Setup → Power → Smart Power On → Enabled**.

Sanity check before involving the puck at all: plug a real USB keyboard into
that port, shut the machine down, press Alt+P. If the machine doesn't power
on, fix the BIOS/port first — the puck can't succeed where a real keyboard
fails.

## 2. Entering wake mode

Any of the usual mode-switch paths, to mode **10**:

- **WebUSB panel**: the *Wake (Alt+P)* mode button. Note the panel cannot
  connect while the puck **is** in wake mode (a bare keyboard has no WebUSB);
  it returns to the puck automatically after firing, or via the escape chord.
- **CDC serial console**: `x10`.
- **A configurable chord**: assign mode 10 to one of the back-4 + D-pad
  chords from the panel.

With the default "always boot Steam" policy this is a **one-shot**: one wake,
then the puck is back to normal. See §5 for making it automatic.

## 3. Using it

1. Enter wake mode (the controller can still be on — arming waits for it).
2. Power the controller off and shut the machine down.
3. To wake: **press the controller's power button.** The puck types the
   hotkey, the machine powers on, and by the time the OS is up the puck is a
   normal puck again and the controller has reconnected by itself —
   single press, end to end.

The escape hatch from wake mode without firing is the **all-four-back-paddles
+ A** chord: reboots straight back to Steam mode.

## 4. The LED tells you what happened

The wake-mode LED is **dark in every steady state** and pulses ~½ s per
hotkey report actually sent (default: 4 pulses over ~5 s). That makes it the
whole diagnostic story:

| You see | It means |
| --- | --- |
| No pulses when the controller connects | The EC never enumerated/configured the keyboard: wrong port, BIOS feature off, or the machine's EC doesn't service USB in S5. |
| Pulses, machine stays off | Reports were delivered but the hotkey parser rejected them: wrong hotkey for your vendor (§6), or the EC wants different press shaping (`WAKE_*` timings in `mode_wake.h`). |
| Pulses, machine powers on | Working as intended. |

## 5. Automatic re-arming (`WAKE_AUTO_REARM`)

By default one wake requires one manual mode-10 switch. For a machine that
always **shuts down** (S4/S5) rather than sleeping, build with:

```bash
make build EXTRA_FLAGS="-DWAKE_AUTO_REARM=1"
```

Then a persistent USB suspend (≥15 s) in any other mode reboots the puck into
wake mode by itself: shut the machine down, and the next controller power-on
wakes it — no interaction with the puck ever again.

**Trade-off, read before enabling:** S3 sleep is *also* a persistent suspend.
With this flag the puck re-attaches as a keyboard mid-sleep, which breaks the
existing wake-from-S3 (controller-connect remote wakeup) path. Use it only on
hosts that fully shut down. Validated end-to-end on the M90q Gen 6.

## 6. Other vendors / tuning the press

Everything the EC sees is a `#define` at the top of `mode_wake.h`:

- `WAKE_MOD` / `WAKE_KEY` — the hotkey (default `LEFTALT` + `P`, Lenovo).
  Change these for Dell/HP/etc. equivalents.
- `WAKE_MOD_LEAD_MS`, `WAKE_HOLD_MS`, `WAKE_RESEND_MS`, `WAKE_REPEAT`,
  `WAKE_REPEAT_MS` — press shaping. The defaults imitate a human keystroke
  (modifier first, long held chord, restated while held) because the M90q's
  EC **ignores** a minimal combined report even though it reads it; other ECs
  are likely no less picky.
- `WAKE_RETURN_MODE` — which personality to return to after firing (default
  Steam).

## 7. What happens behind the scenes (and what was hard)

Full flow: arm (3 s of RF-link silence, so switching modes with the
controller still connected can't type into a live session) → fire on the
first controller connect → settle → reboot into the return mode with a
persisted one-shot **handoff grace**. During the grace (until the OS actually
enumerates the puck, or 90 s), two standing behaviors that would otherwise
misread a POSTing machine as "host went to sleep" hold their fire: the
suspend-triggered controller power-off, and `WAKE_AUTO_REARM` itself. The
post-fire boot also skips the USB mount wait and starts RF beacons
immediately, so the controller that triggered the wake reconnects in well
under a second instead of giving up during POST. Design details and the
hardware findings behind each decision are in `mode_wake.h` / `mode_wake.cpp`
comments.
