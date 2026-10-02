# Switch Pro profiles, trackpad D-pad and HD Emulation

## Profiles and trackpads

The configurator supports seven Switch Pro back-button profiles. Each profile stores L4/R4/L5/R5 mappings; gyro, trackpads, rumble and other settings are shared. Select a profile to edit or activate it, and assign profiles to button shortcuts.

Trackpad mapping includes D-pad on touch and D-pad while clicked, including diagonals. Trackpad D-pad click feedback has a separate enable control.

## Rumble

HD Emulation combines Punchy grip rumble and game-frequency trackpad tones in Switch Pro mode. The two strengths are independent. Other emulated modes use normal grip rumble for this style; Steam retains native haptic relay. Normal, Mono, Heavy, Light, Swapped motors, Punchy and Soft retain their original behavior.

Each pad follows the stronger weighted low/high band rather than mixing two frequencies. Tones last 20 ms and refresh every 16 ms. A stale report, silence, suspend or disabled rumble stops output. Short finite tones bound output even if an RF stop is lost.

The trackpad command ceiling is −15 dB from 250–300 Hz, returning gradually to −3 dB over 230–250 and 300–320 Hz. This frequency-dependent ceiling came from controller calibration and gameplay testing; it is not a measured acoustic limit or a guarantee for every controller. Effects already quieter than the ceiling keep their level. Grip pitch remains standard rumble. Packed rumble timing and simultaneous frequency bands are approximated.

Per-type Rumble controls disable game rumble. Trackpad strength 0 disables HD pad output. Native pad feedback remains separate.

## Shortcuts

Choose Quick Access or all four back buttons as modifier, and Controller modes or Switch profiles as the shortcut behavior. Shortcuts, D-pad haptic shortcuts, confirmation pulses and the Quick Access + Select action can be disabled separately.

| Modifier + button | With D-pad haptic shortcuts enabled |
| --- | --- |
| Left | Cycle three selected rumble styles |
| Up | Cycle three trackpad strength values |
| Down | Cycle three grip strength values |
| B / X / Y / Right | Assigned mode or Switch profile |
| A | Return to Steam |

Trackpad steps accept 0–500%; grip steps accept 10–500%, in 2% increments. Default steps are 200/300/500%. Duplicate or unsorted values retain separate slot positions.

Left/Up/Down confirm the slot with one, two or three pulses on both trackpads only: 250 ms per pulse, 150 ms gaps. Other shortcuts confirm with one pulse. USB mode changes wait for enabled feedback without blocking the input loop. Disabling feedback restores immediate mode transitions.

Disable D-pad haptic shortcuts to restore Left/Up/Down to configurable mode or profile assignments. Seven non-A profile shortcuts are available in that configuration; with haptic shortcuts enabled, four remain. Unassigned profile shortcuts do nothing. A always returns to Steam.

Quick Access + Minus/Select/Menu has an independent Switch Pro mapping, defaulting to Take screenshot. It works with either modifier setting. Minus is suppressed until release, including if Quick Access is released first. Holding Capture retains the console's normal capture hold behavior.

Quick Access retains its normal per-type mapping when it is not reserved as the enabled modifier. Selecting the four-back modifier suppresses the four paddles while held together; otherwise normal paddle mappings apply.

Controller shortcut changes are live. Use Save shortcut settings, or edit a configurator setting, to retain them across restarts. Configurator edits save automatically.

## Upgrade and legacy behavior

Configuration fields are appended, and reserved experimental bytes keep the saved layout compatible with development builds. Existing pairing, strengths, profiles and mode assignments remain readable. Development waveform selectors and calibration commands no longer control haptics.

Settings, bonds and lizard maps use verified temporary-file writes before replacement. Nonblank storage is not automatically formatted after a failed mount. The configurator reports unavailable storage and failed saves.

The v36 status payload is 251 bytes, fitting the 256-byte vendor FIFO with its two-byte frame header. Backups include shortcut flags, rumble presets, strength steps and active slots; final-feature backups are rejected on older firmware before writes begin.

To use the original shortcut arrangement, select All four back buttons and Controller modes, disable D-pad haptic shortcuts, and optionally disable confirmation feedback and Quick Access + Select. Original controller modes and per-type controls remain available.

## Host verification

```sh
python3 tests/hd_rumble/run.py
python3 tests/switch_pro_profiles/run.py
python3 tests/final_webusb/run.py
python3 tests/shortcut_modes/run.py
python3 tests/storage/run.py
npm install --prefix /tmp/openpuck-ui-tests jsdom@26
NODE_PATH=/tmp/openpuck-ui-tests/node_modules node tests/switch_pro_profiles/configurator.cjs
make check
make build
```

The C++ helper tests require g++ and use AddressSanitizer/UndefinedBehaviorSanitizer. Configurator tests use jsdom. Host checks cannot validate actuator feel or console pairing; test those on physical hardware.
