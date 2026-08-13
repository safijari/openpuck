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
