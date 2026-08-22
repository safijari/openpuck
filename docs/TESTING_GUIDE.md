## What to test before merging a change

- Can you pair a controller?
- Does the controller connect?
- Does the controlelr work normally in Steam Controller mode, do the haptics work, is the gyro working?
- Can you shut the controller off using Steam + Y?
- Does the controller reconnect?
- Can you change modes via the chords? (back4 + A/B/X/Y **and** back4 + each D-pad direction; the chorded
  press must not reach the game, and a D-pad reassignment made in the panel must take effect)
- Can you change modes via webusb?
- Does rumble still work in the translated modes, and do the panel's Rumble style/strength settings
  change how it feels? (heavy vs light drive different motors, so they must feel clearly different;
  the Test rumble button must buzz and then stop on its own)
- Do the modes work as expected (including gyro in gyro modes)
- Is signal quality shown?
- Is battery level shown and correct?
- Is the polling rate and response rate the same? Is it around 250hz?

### Testing the DirectInput mode (mode 11)

Windows has a built-in DirectInput tester: run `joy.cpl` (or Settings → Bluetooth & devices → Devices → More
devices → *Set up USB game controllers*). **Two** entries should appear — that's the point of the mode; one
carries the sticks/triggers/hat and 26 buttons, the other the trackpad and gyro axes. Open *Properties* on
each and watch the axis crosshairs/sliders move. Note that joy.cpl's page only draws the first few axes; a
tool like [DIView](https://github.com/pklaus/DIView) or *GamepadTester* lists every axis a device reports.
Things to confirm:

- both devices enumerate (if only one does, Windows merged the collections — see the note in `mode_dinput.cpp`)
- trackpad axes hold their position after you lift a finger, and re-centre when you *click* that pad
- gyro axes rest at centre and move with rotation
- all 26 buttons register, one per physical input, with no duplicates

DirectInput is Windows-only. On Linux the same descriptor shows up through evdev (`evtest`, `jstest`), but how
the two collections are split is kernel-dependent — use SInput mode there.

### Testing the SInput mode (mode 12)

SInput needs a host with SDL's SInput driver (SDL 3.4+ / a current Steam client). Practical checks:

- **Steam**: Settings → Controller → the device is listed; open its test/calibration page and confirm face
  buttons land on the right glyphs (an A/B or X/Y swap means the face bit order is wrong), Start vs Select,
  the paddles show as paddle1-4, both analog triggers move independently, gyro/accel report, and *both*
  trackpads show as touchpads.
- **battery**: the percentage/charging state comes from the controller's own 0x43 report, so it should match
  what the panel shows in Steam mode.
- **rumble**: any game/tool that rumbles (Steam's controller test has a haptic test) — SDL sends the ERM-style
  haptic command, which the firmware turns into the usual 0x80 rumble relay.
- **SDL directly**: SDL's own `testgamepad` (from an SDL 3.4+ build) prints axes, sensors and touchpads, and
  is the least ambiguous way to see exactly what the driver decoded.
