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

- **WebUSB panel**: the *Keyboard wake (power-on)* mode button. Note the panel cannot
  connect while the puck **is** in wake mode (a bare keyboard has no WebUSB);
  it returns to the puck automatically after firing, or via the escape chord.
- **CDC serial console**: `x10`.
- **A configurable chord**: assign mode 10 to one of the back-4 + D-pad
  chords from the panel.

Entering wake mode is always a **one-shot**: one wake, then the puck returns
to your normal personality (the persisted mode if you use *persist last
mode*, Steam otherwise). See §5 for making it automatic. Arming takes a few
seconds — the mode requires ~5 s of observed controller-radio silence before
a connect counts as a wake gesture, so switching modes with the controller
still on can't type into your live session.

## 3. Using it

1. Enter wake mode (the controller can still be on — arming waits for it).
2. Power the controller off and shut the machine down.
3. To wake: **press the controller's power button.** The puck types the
   hotkey, the machine powers on, and by the time the OS is up the puck is a
   normal puck again and the controller has reconnected by itself —
   single press, end to end.

**Leaving wake mode.** The **all-four-back-paddles + A** chord on a connected
controller reboots straight back to Steam mode, and works whether the machine
is on or off (wake mode is exempt from the usual
no-mode-switch-while-suspended rule in both directions — the chord is also
how you *enter* mode 10 with the machine already off). Timing matters,
though: the chord only escapes **without firing** while the controller that
was connected at the mode switch stays connected (the mode never arms with a
link up). Once armed, a *fresh* controller connect is the wake gesture and
fires immediately — on a running machine that types the hotkey once into the
session and bounces back to the normal puck by itself, which is the other,
cruder way out. Note the trigger is *any* bonded controller connecting; with
two controllers, one left powered on prevents arming until it, too, goes
quiet.

## 4. The LED tells you what happened

The wake-mode LED is **dark in every steady state** and pulses ~½ s per
hotkey press (default: 4 pulses over ~5 s). In wake mode the LED is reserved
for presses — the connect-time remote-wakeup pulse other modes show is
suppressed so it can't be mistaken for a fire. That makes it the whole
diagnostic story:

| You see | It means |
| --- | --- |
| No pulses when the controller connects | First rule out timing: the mode arms ~8 s after the link goes quiet (2.5 s radio bring-up + 5 s observed quiet), so a power-press within ~8 s of shutdown, or a second bonded controller still powered on, means it simply was not armed yet — power the controller off, wait ~10 s, try again. If timing is ruled out: the EC never enumerated/configured the keyboard — wrong port, BIOS feature off, or the machine's EC doesn't service USB in S5. |
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

**Sleep vs shutdown:** the firmware tells them apart by the host's own hand —
an OS going to sleep arms USB remote wakeup on the puck first (that's what
makes the existing wake-from-S3 feature work), and such suspends never
re-arm, so sleeping the machine leaves the puck a normal puck and
controller-wake-from-sleep keeps working. A shutdown never arms remote
wakeup, so it re-arms as intended — even though some ECs (the M90q's
included) re-arm the feature themselves when they take the port over in S5;
the decision is sampled from the OS's own suspend, so that doesn't confuse
it. A wake that *fails* (machine never boots) re-arms again when the 90 s
handoff grace runs out (~90–105 s after the fire), so a later press still
works. Three caveats: if your OS has wakeup *disabled* for the device
(Linux: `power/wakeup` in sysfs), its sleep looks like a shutdown and the
puck will re-arm mid-sleep — recover with the escape chord or a replug, or
enable device wakeup. A host that arms remote wakeup once but never clears
it (Linux clears on resume; other OSes untested) would make shutdowns look
like sleeps — if auto-rearm silently stops re-arming on your machine, that's
the signature. And if the *puck itself* restarts while the host is asleep
(replug, firmware flash, watchdog reset), a sleeping host is indistinguishable
from an off one — the puck re-arms ~45 s later and the resumed session finds
the wake keyboard instead of a controller; escape-chord or replug recovers.
Validated end-to-end on the M90q Gen 6.

## 6. Other vendors / tuning the press

- **The hotkey is runtime config**: the *Wake hotkey* row on the WebUSB
  panel sets the modifier(s) + key the mode types (default `Alt+P`, Lenovo).
  Change it there for Dell/HP/etc. equivalents — no rebuild needed. The
  setting persists on the puck, and the compiled default lives in
  `mode_wake.h` (`WAKE_MOD_DEFAULT` / `WAKE_KEY_DEFAULT`).
- Press *shaping* stays a `#define` at the top of `mode_wake.h`:
  `WAKE_MOD_LEAD_MS`, `WAKE_HOLD_MS`, `WAKE_RESEND_MS`, `WAKE_REPEAT`,
  `WAKE_REPEAT_MS`. The defaults imitate a human keystroke
  (modifier first, long held chord, restated while held) because the M90q's
  EC **ignores** a minimal combined report even though it reads it; other ECs
  are likely no less picky.
- `WAKE_RETURN_MODE` — which personality to return to after firing. The
  default (`0xFF`) means "the normal boot policy": your persisted mode if
  *persist last mode* is on, Steam otherwise. Set a concrete mode to force
  one.

## 7. What happens behind the scenes (and what was hard)

Full flow: arm (~5 s of observed RF-link silence, so switching modes with the
controller still connected can't type into a live session) → fire on the
first controller connect → settle → reboot into the return mode with a
persisted one-shot **handoff grace**. During the grace (up to 90 s; it ends
early once the OS has been stably up for 20 s — a bar BIOS POST enumeration
never clears, so a quick shutdown after a wake behaves normally; the one
exception is parking in BIOS setup for 20 s+ right after a wake, which also
ends the grace early), two standing behaviors that would otherwise
misread a POSTing machine as "host went to sleep" hold their fire: the
suspend-triggered controller power-off, and `WAKE_AUTO_REARM` itself. If the
machine never boots, the grace expires and the controller is powered off
after all — the battery is still saved. The
post-fire boot also skips the USB mount wait and starts RF beacons
immediately, so the controller that triggered the wake reconnects in well
under a second instead of giving up during POST. Design details and the
hardware findings behind each decision are in `mode_wake.h` / `mode_wake.cpp`
comments.
