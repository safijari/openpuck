# DualSense mode: audio haptics

Notes on how DualSense mode delivers DualSense "HD haptics" to the Steam Controller 2 and what that needs from the USB
presentation, the Linux audio stack and Proton. It also records how the Steam Controller 2 haptic commands behave, as
measured on hardware. Written 2026-09-23, from work on the `dualsense-haptic` branch.

Status at the time of writing:

| Item | State |
|---|---|
| Stellar Blade (Sony PC port, libScePad), GE-Proton11-7, Steam Input off | Input + audio haptics work, both actuators. **Soft running footsteps are often not felt in play**, although the controller renders every one when the firmware's commands for them are replayed (§5); cause unknown |
| Hi-Fi Rush, GE-Proton11-7 | Audio haptics work in rumble and split styles; split feels best |
| Split style (`AUDIO_STYLE_SPLIT`) | Works (Hi-Fi Rush, Stellar Blade). Rumble and a tone on the same actuator play together |
| Wave style (`AUDIO_STYLE_WAVE`, the default) | Works (Stellar Blade: smoother than tone, a little soft). The haptic channels streamed as 4 kHz PCM to the grip actuators |
| Tone style (`AUDIO_STYLE_TONE`, the default before wave) | Works, but deep effects play as higher tones, and it misses the deep feel split gives |
| Control Resonant, GE-Proton11-7, Steam Input off | Audio haptics work (wave). Reported working on Proton-Wineland `wineland-11.0-20261005`, and on older Wineland with `PROTON_USE_PIPEWIRE=0` ([proton-cachyos#86](https://github.com/nanomatters/proton-cachyos/issues/86)); not tested here (§4) |
| FFXIV (XIVLauncher) | Input + audio haptics work (GE-Proton10-34; GE-Proton11-7 after repairing the prefix, §9) |
| Windows | Untested |
| PS5 console | Not supported: the console authenticates controllers and the puck can't answer |

## 1. How it works

A DualSense drives its two voice-coil actuators from audio: its USB audio function is a 4-channel 48 kHz output, and
games put the haptic waveforms on channels 3 and 4 (left and right actuator). Channels 1 and 2 go to the speaker or
headphone jack.

In DualSense mode (`MODE_PS5`, and the clean `MODE_PS5_GAME`) the puck presents the same audio function
([mode_ps5_audio.cpp](../OpenPuck/mode_ps5_audio.cpp)). It reads channels 3 and 4, turns them into Steam Controller 2
haptic commands, and relays those over RF like any other haptic (§5). Channels 1 and 2 are accepted and
discarded.

## 2. USB presentation: what must match a real DualSense

Two different consumers check how closely the puck resembles a real pad: PipeWire's DualSense profile (§3) and Sony's
PC controller library, libScePad (§4). Reference: the real CFI-ZCT1W descriptors in
[github.com/nondebug/dualsense](https://github.com/nondebug/dualsense).

| Property | Real DualSense | Puck | Why it matters |
|---|---|---|---|
| Interfaces | 0 AC, 1 AS out (4ch), 2 AS in (2ch mic), 3 HID | same | Linux names the sound card after the first audio interface (`…-00`); Wine reports the HID interface number to games (`MI_03`) |
| Mic streaming interface | 2ch 48 kHz | 2ch 48 kHz, sends silence | Required: without it PipeWire rejects every profile of the DualSense UCM config (§3) |
| Output channel config | `wChannelConfig 0x0033` (FL FR RL RR) | same | Gives the quad channel mask GE-Proton checks for |
| USB serial string | none (`iSerial 0`) | none in `MODE_PS5_GAME`; kept in `MODE_PS5` | Names match real hardware exactly: `…DualSense_Wireless_Controller-00`. `MODE_PS5` keeps the serial because its mount-count suffix makes Windows re-read the configuration when a controller connects |
| `bcdDevice` | 1.00 | 1.00 | Games see it as the HID `VersionNumber` |
| HID report descriptor | 273 bytes, feature reports up to `0xF5` | byte-for-byte copy | See below |
| Feature `0x09` (pairing) | MAC, `08 25 00`, host MAC | same shape | See below |

The serial string is removed with a linker wrap of `tud_descriptor_device_cb` (in
[mode_ps5.cpp](../OpenPuck/mode_ps5.cpp), flag in the [Makefile](../Makefile)), because the Adafruit core always
reports one.

**libScePad and the HID identity.** With a truncated report descriptor (ending at feature `0x22`), a zero-filled
pairing report and `bcdDevice` 1.10, Stellar Blade showed a PS5 controller layout but ignored all input. Wine traces
showed libScePad opening the pad, reading its attributes, strings, feature `0x09` and feature `0x20`, then closing it
and rescanning, about 300 times, and never reading calibration (`0x05`). Matching all three of the items above fixed
input on Valve Proton Experimental and GE-Proton11-7. Which of the three libScePad actually checks is unknown.

**Feature reports** are answered the way a real USB pad answers them: in full at the size the descriptor declares, or
not at all (a stall), never with a short reply. `0x05` (calibration), `0x09` (pairing), `0x20` (firmware info),
`0x22`, `0x81`, `0x83`, `0x85`, `0xE0`, `0xF1`, `0xF2` and `0xF5` are answered, with a real pad's values where they
are known and zeros otherwise; every other id stalls. The audio function's terminal and unit IDs, endpoint
attributes and packet sizes also match a real pad's.

## 3. Linux host setup

Tested with alsa-ucm-conf 1.2.16.1, PipeWire 1.6.8 and WirePlumber 0.5.17.

**No WirePlumber config is needed.** alsa-ucm-conf matches `054c:0ce6` to `USB-Audio/Sony/DualSense-PS5.conf`. Its
**Direct** profile (priority 2000) wins over the split "Default" profiles (priority 200) and gives a 4-channel sink:

```
alsa_output.usb-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-00.Direct__Direct__sink
s16le 4ch 48000Hz, channel map front-left,front-right,rear-left,rear-right
```

The Direct profile also opens a 2-channel capture device. Before the puck had a mic interface, every profile failed
the probe on "input PCM open failed", which is why an earlier WirePlumber config disabled UCM and renamed the sink. That
rename is what stopped GE-Proton from finding the haptics output. To see how PipeWire probes a card:

```sh
spa-acp-tool -vvvvv -c <card-number> list-profiles
```

**Set the controller output to 100% once.** WirePlumber gives a new output a default volume of 0.064 (about 6%;
`device.routes.default-sink-volume`). The Direct profile has no hardware mixer, so PipeWire scales the samples
themselves, and the haptics would arrive at about 6% strength. WirePlumber saves the volume per device and route in
`~/.local/state/wireplumber/default-routes`:

```sh
wpctl set-volume <sink-id> 1.0
```

In `MODE_PS5_GAME` the device name is stable, so this is a one-time step. In `MODE_PS5` the serial contains the mounted
controller count, so it's once per count. A real DualSense needs the same step. Setting only the haptic channels
leaves the speaker channels at their default:

```sh
pactl set-sink-volume alsa_output.usb-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-00.Direct__Direct__sink \
      40% 40% 100% 100%
```

**WirePlumber also remembers the card profile per card name.** If the card was ever switched to a "Default" profile
(for example while testing a game), it comes back on that profile, which has only a mono speaker sink and no haptic
channels. Switch it back once:

```sh
pactl set-card-profile alsa_card.usb-Sony_Interactive_Entertainment_DualSense_Wireless_Controller-00 Direct
```

**Optional hook:** `make install-wireplumber` installs
[60-openpuck-dualsense.conf](../tools/wireplumber/60-openpuck-dualsense.conf) and
[openpuck-dualsense-haptics.lua](../tools/wireplumber/openpuck-dualsense-haptics.lua). For a Direct output route
WirePlumber has not saved a volume for, it starts the haptic channels (RL/RR) at 100% and leaves the speaker channels,
the mic and saved volumes alone. Not yet installed or verified on hardware.

## 4. Proton

Games reach the controller's audio in one of two ways:

- **By name** (e.g. Hi-Fi Rush): they open the audio endpoint whose name contains "Wireless Controller" (a real pad's USB product string is "DualSense Wireless Controller"). Works on any Proton build.
- **Through libScePad** (Sony PC ports, e.g. Stellar Blade, Spider-Man): the library links the HID device to its
  audio endpoint through the Windows device tree (container IDs). Stock Wine and Valve Proton don't link them (checked
  in Valve's `proton_11.0` and `experimental_11.0` branches), so these games get no haptics even with a real
  DualSense.

The [proton-ds5-haptic](https://github.com/xzn/proton-ds5-haptic) patches add that link. They ship in GE-Proton from
**11-2**. From **11-6**, GE-Proton also routes libScePad haptics through PipeWire, to a sink it recognizes by name: one
with `api.alsa.split.name` (UCM Default profile) or one whose node name contains `Direct__Direct__sink` with a quad
channel mask. Both names come from the stock UCM profiles.

**Proton-Wineland** (nanomatters/proton-cachyos) uses Wine's PipeWire audio driver (`winepipewire.drv`) by
default. Up to `wineland-11.0-20260930` that driver dropped DualSense USB audio haptics: Control Resonant played
none, while `PROTON_USE_PIPEWIRE=0` (back to winepulse) made them work.
[`wineland-11.0-20261005`](https://github.com/nanomatters/proton-cachyos/releases/tag/wineland-11.0-20261005)
fixes it ([#86](https://github.com/nanomatters/proton-cachyos/issues/86)), so current builds need no variable. On
2026-10-05 Control Resonant was also seen with no haptics on Wineland 20260930 with Steam Input on; GE-Proton11-7
with Steam Input off fixed it, but that changed both at once, so it doesn't show which was needed.

For libScePad games, **disable Steam Input** for the game. With it on, the game sees Steam's virtual Xbox pad
(XInput) and libScePad never finds a DualSense. Don't set `PROTON_SONY_DUALSENSE_AS_DUALSHOCK4`, which hides the
DualSense.

**Debugging:**
- Run with `PROTON_LOG=1 WINEDEBUG=+pulse %command%`. GE-Proton logs `DualSense raw PCM channel peaks: a, b, c, d` for
  the four channels it writes, where c and d are the haptic channels.
- Add `+hid,+setupapi` to see whether the game opens the HID device. These logs grow quickly; one run reached 171 MB.
- To record what reaches the controller, **pass the channel map explicitly**. `parec --channels=4` alone uses
  PulseAudio's 4-channel default (FL, FC, FR, RC), and PipeWire then downmixes both haptic channels into one:
  ```sh
  parec -d <sink>.monitor --channels=4 --channel-map=front-left,front-right,rear-left,rear-right \
        --format=s16le --rate=48000 > haptics.raw
  ```

## 5. Steam Controller 2 haptic commands (measured)

Output reports `0x80`–`0x89` reach the controller unchanged (see [PROTOCOL.md](PROTOCOL.md)). Payloads follow SDL's
`src/joystick/hidapi/steam/controller_structs.h`:

| Report | Payload |
|---|---|
| `0x80` rumble | `[type][intensity u16][left speed u16][left gain s8][right speed u16][right gain s8]` |
| `0x81` pulse | `[side][on_us u16][off_us u16][repeat u16]` |
| `0x82` command | `[side][command][gain_db s8]` |
| `0x83` tone | `[side][gain_db s8][frequency u16][duration_ms u16][lfo_freq u16][lfo_depth u8]` |

Findings, measured with the controller's IMU (§6) or by feel where noted:

- **Sides (tone):** `0` = left actuator, `1` = right; each is stronger under its own trackpad. `2` felt centered (both),
  and `3` produced nothing (both by feel). The controller firmware shows why: 2 is both touchpads, and 3-5 are the
  left, right and both **grip** actuators, which are hard to feel by hand (PROTOCOL.md section 9.1). Left and right commands are **independent**: a command for one side
  doesn't interrupt the other.
- **`0x80` rumble** imitates a spinning motor at every setting. Types 3, 4 and 5 felt identical. `speed` behaves like
  strength; `gain` had little effect. It measures as a fluctuating vibration (IMU 2,400–4,300 at speed `0xA000`,
  against a steady ~2,200 for a 0 dB tone), which users feel as rough. Re-sending identical values every 20 ms or
  every 60 ms, or sending once, felt the same.
- **`0x83` tone** is smooth. It restarts on every command, but:
  - an unchanged tone re-sent every 60 ms, each lasting 200 ms, is as smooth as one long tone (same IMU dip rate,
    0.3–0.5 per second, which is measurement noise);
  - strength steps of about 3 dB a few times a second trace cleanly; 1 dB steps every 20 ms feel choppy;
  - frequency changes make the vibration swing, so change frequency rarely;
  - felt down to at least −48 dB, with the controller still getting softer beyond that; 40–250 Hz are all distinct,
    and 350 Hz is very weak.
- **Stopping a tone:** `duration 0` is ignored (the tone keeps playing). `duration 1`, `0x82` command 0, and a tone
  simply ending all fade out over about 100–150 ms. `gain_db −128` cuts it within about 25–50 ms.
- **The radio relay doesn't acknowledge or retry.** Single commands are sometimes lost, so repeat stops, and give
  tones a finite duration so a lost update can't leave one playing.
- **Short effects** (the firmware's commands for 28 of Stellar Blade's running steps, 40–60 ms each, replayed in
  their original timing on one actuator; IMU about 18 at rest):
  - `0x80` rumble at about 21% speed: all 28 registered at 880–1,740 (three runs) and fell to mostly 60–150 between
    steps.
  - `0x83` tones at about −8 dB: with the frequency then played, 25 of 28 registered, and every step played at
    40 Hz measured 36–124. With the fixed estimate (§7), mostly 125 Hz: 28 of 28, 266–1,608. The IMU measures
    acceleration, which falls with frequency at the same drive, so this overstates how much weaker 40 Hz feels.
  - Both on the same actuator at once (split style): about as strong as rumble alone, and stronger where the tone
    was strong (1,849 against 1,167 rumble and 1,608 tone alone). Neither cuts the other off.

## 6. Measuring haptics on hardware

In Steam Controller (puck) mode the puck passes haptic output reports straight through, so haptic commands can be sent
from the PC. The puck only relays haptics that arrive on the **connected controller's slot**: HID interfaces 2–5 are
slots 0–3.

To measure the result, turn the controller's IMU on and read the accelerometer from input report `0x42` (bytes
`0x22`–`0x27`, three signed 16-bit values; about 230 reports per second). With the controller flat on a table and
hands off, the RMS deviation of the accelerometer is about 20 at rest and about 2,200 for a 0 dB tone. Steam owns
`IMU_MODE` in this mode, so set it back afterwards.

```python
import os, fcntl, struct
HIDIOCSFEATURE = lambda n: 0xC0000000 | (n << 16) | (ord('H') << 8) | 0x06
fd = os.open('/dev/hidrawN', os.O_RDWR)  # the connected slot's interface

def tone(side, gain_db, hz, ms):
    os.write(fd, bytes([0x83, side]) + struct.pack('<bHHHB', gain_db, hz, ms, 0, 0))

def imu_mode(value):  # feature 0x01: SET_SETTINGS_VALUES (0x87), IMU_MODE (0x30); 7 = on, 0 = off
    buf = bytearray(64); buf[:6] = bytes([0x01, 0x87, 3, 0x30, value, 0])
    fcntl.ioctl(fd, HIDIOCSFEATURE(64), bytes(buf))

imu_mode(7)
tone(0, 0, 150, 1000)                  # 1 s, left actuator, 150 Hz, 0 dB
r = os.read(fd, 64)                    # keep r[0] == 0x42: accel = struct.unpack_from('<hhh', r, 0x22)
imu_mode(0)
```

Measuring beats judging by feel: differences a tester couldn't feel reliably showed up clearly in the numbers.

To test a game's effects, record the haptic channels (§4), port the firmware's decisions (§7) to a script that turns
the recording into the command list with timestamps, and replay that list with the IMU on. This separates what the
firmware decides from what the controller renders, without a game running, with the controller flat on a table.

## 7. Firmware: from audio to haptics

In [mode_ps5_audio.cpp](../OpenPuck/mode_ps5_audio.cpp) (`processAudioSamples`, `ps5AudioTask`):

1. **Level per 20 ms tick,** from the sum of squares of each haptic channel (RMS, scaled by √2 so a sine reads as its
   peak). Using the peak of the latest 1 ms USB packet instead gave random strengths: 1 ms is a tenth of a 100–150 Hz
   haptic cycle.
2. **Envelope: instant rise, halving fall.** Noise-like textures, such as Stellar Blade's sprint, still jump ±25%
   between ticks as RMS; the halving fall halves that jitter while hits land at full strength on their first tick.
3. **Auto gain** (default, `g_audioHapticGain == 0`): a reference level plays at full strength. It jumps to any louder
   peak and sinks by 1/1024 per tick (about a 20 s time constant), never below 8192, which caps the boost at 4×. A
   manual 10–500% gain uses full scale as the reference instead. Wave style skips this step (see below).
4. **Drive:** √(level / reference) × gain, 1.0 = full strength. Linear left Stellar Blade's footsteps (40 ms pulses at
   about 5% of full scale) at 10–17% rumble or −15 to −20 dB tone, which isn't felt; the root puts them at 35–45%.
5. **Output style** (`g_audioHapticStyle`):
   - **Rumble:** the drive as the `0x80` rumble speed. Gate 400 (about 1.2% of full scale).
   - **Tone:** per actuator, a `0x83` tone with gain = 20·log10(drive), −60 to 0 dB. The frequency
     is zero crossings (±64 hysteresis) per frame that carried signal (beyond ±64), over the last three ticks that
     had a crossing; until one has, a new tone keeps the band it last played. Counting per tick instead read a step
     that starts late in a tick, and the silence after it, as 40 Hz: Stellar Blade's running steps then measured a
     quarter or less of their 125 Hz response on the IMU, and 3 of 28 didn't register. The result is snapped to 40, 50, 63, 80, 100,
     125, 160, 200 or 250 Hz, and a new step must hold for two ticks. Hits (+6 dB) are sent at once; changes of 2 dB
     or more at most every 40 ms; otherwise the tone is refreshed every 60 ms with a 200 ms duration. An ending effect
     is cut with −128 dB, sent twice. Gate 100. Zero crossings follow the fastest strong component, so a deep effect
     with higher texture on top plays high: Stellar Blade's dash has 38% of its energy below 80 Hz but reads 125–300 Hz.
   - **Split:** each haptic channel goes through an 80 Hz 2nd-order Butterworth low-pass. The part below drives that
     side's `0x80` rumble speed (gate 400), and the rest (signal minus low-pass) drives the tone as above, including
     its zero crossings (gate 100).
   - **Wave** (default): the haptic channels themselves, through a 300 Hz low-pass (two cascaded 2nd-order Butterworth
     sections; in a feel test 500 Hz was still a little harsh on strong hits), decimated to 4 kHz,
     scaled linearly with no auto gain, as a real DualSense plays them: 100% (and Auto) puts int16 full scale at 40%
     of u-law full scale. Half matched a real pad's strength in FFXIV (2026-10-04) but felt too strong in most other
     games, including Control Resonant, where 80% of that felt right (2026-10-05). Then u-law encoded and sent as `0x88` stereo PCM frames of 31
     samples (129 frames/s), left channel to the left **grip** actuator (`0x88` is a grip stream). Streams while either channel's envelope is above
     gate 100 and for 300 ms after; `0x86 {2, 2, 9}` sets the format at each start and every second. The controller
     pre-buffers (24 ms with the short third frame, PROTOCOL.md), so it starts about 20 ms later than a tone. No `0x80` rumble from the audio. See PROTOCOL.md
     section 9.1 for the measured PCM behaviour.

The game's ordinary rumble (output report `0x02`) always goes through `0x80`, in every style. `hapticUpdateRumble`
adds it to the audio rumble (rumble and split styles) and sends one frame.

Web panel and config (protocol v22): on/off is field 31 (blob `p[197]`), gain is field 30 (percent/2, 0 = Auto;
blob `p[196]`), style is field 88 (blob `p[198]`: 0 rumble, 1 tone, 2 split, 3 wave). The panel's style button
cycles rumble, tone, split, wave. All three are saved in `cfg.bin`.

**Grip limiter** (Rumble card, protocol v22): the knee of the soft limiter on the grip PCM stream, as a
percent of full scale: 50, 60, 70 (default), 80, 90, or 100 = off. Below the knee the waveform passes unchanged;
above it, peaks are rounded off toward full scale instead of clipping, which plays as a pop. Lower is smoother on
the strongest hits, higher keeps more of their punch. One setting (`g_hapticLimitKnee`, `hapticSoftLimit` in
haptics.h) covers wave haptics and Switch Pro HD rumble (PROTOCOL.md §9.5). Field 115, blob `p[199]`, saved in
`cfg.bin`.

## 8. Pitfalls

- **Web panel field IDs:** fields 40–75 are the per-type settings (`40 + type·9 + k`) and are handled before the
  `switch` in `webusb_config.cpp`. 80–87 are the trackpad→stick mapping. A new field in those ranges silently edits
  another setting.
- **Stale Wine devices:** a Proton prefix keeps HID device entries from earlier modes and serials. They show up in
  enumeration, but no game was seen opening one.
- **GE-Proton issue #714** (no controller input with Steam Input off on 11.2+) did not reproduce once the puck matched
  the real HID identity.
- **Recording without a channel map:** a recording made with `--channels=4` alone puts both haptic channels into one
  (§4). It still shows timing and levels, but not which side an effect is on.
- **Logging build:** a `-DOPK_LOG=1` build of this branch boot-looped in DualSense mode. Cause unknown. Recover by
  double-tapping RST (or shorting RST to GND twice) and dropping a release UF2 on the bootloader drive.
- **Formatting:** CI pins clang-format 18. A newer clang-format (22 was tried) formats some lines differently, e.g.
  `(int8_t) - *sign`, and fails CI's check.

## 9. FFXIV

Works (2026-10-04) with a real DualSense and with the puck in DualSense mode, on GE-Proton10-34 and, after the prefix
repair below, on GE-Proton11-7: buttons, audio haptics, and the confirmation pulse when "PlayStation controller
support" is switched on in the gamepad settings. Setup: XIVLauncher-RB with its managed Proton, USB, PipeWire's stock
Direct profile, no `PROTON_SONY_*` variables.

How FFXIV uses the pad, from Wine traces (`WINEDEBUG=+hid,+dinput,+mmdevapi`):

- **Buttons come through DirectInput.** The game lists joysticks, runs the usual "is this an XInput device?" check
  (a WMI query of `Win32_PnPEntity` for a device ID with `IG_` and the pad's VID/PID), then opens the pad's
  DirectInput device and polls `GetDeviceState`. The device's button map is saved in `FFXIV.cfg`, under
  `<GamePad Settings>`, as `InstanceGuid`, `ProductGuid` and `Alias`, for one pad identified by its DirectInput
  instance GUID.
- **DualSense features come through raw HID.** It reads features `0x09`, `0x20` and `0x05`, writes output report
  `0x02` (lightbar, player LEDs, triggers, speaker routing), and opens a 4-channel audio stream for the haptics.

On Wine 11 (GE-Proton 11.x, Proton-cachyos 11) the same prefix failed in two ways that GE-Proton10-34 doesn't
trigger. Neither depends on the controller: a real DualSense failed the same way as the puck. The firmware, the
product string and the Steam Runtime version (steamrt4 vs steamrt3) were not the cause.

1. **Stale XInput devices hide the pad.** A prefix that ever exposed a Sony pad through XInput (older Proton, Steam
   Input, `PROTON_SONY_HIDRAW_XINPUT=1`) keeps `WINEXINPUT\VID_054C&PID_0CE6&IG_xx` device keys. Wine 11's WMI lists
   them even when nothing is plugged in (`wine wmic path Win32_PnPEntity get DeviceId` shows every device the prefix
   has seen). The game's XInput check finds them, so it releases the DualSense's DirectInput device right after
   `GetDeviceInfo` and `GetCapabilities`: no buttons, and no haptic stream. In a trace, that's
   `dinput_device_Release` straight after a `wbemprox` query.
2. **A saved button map points at nothing.** Wine 11 saves DirectInput instance GUIDs in the registry and reuses
   them across runs, so the pad gets the same GUID that `FFXIV.cfg` holds, and the game applies the saved `Alias`.
   That map was recorded while the pad looked different, so presses reach DirectInput (`hid_joystick_read`) while
   the game's pad state (Dalamud's `/xldata`, gamepad tab) stays at zero. Wine 10 gives the pad a different instance
   GUID, so the saved map was never applied there. Haptics work in this state.

**Repair**, with the game closed and both files backed up:

1. In the prefix's `system.reg` (XIVLauncher: `~/.local/share/dev.goats.xivlauncher/protonprefix`), delete every
   key whose path contains `WINEXINPUT\\VID_054C&PID_0CE6`. This applies to any game and prefix.
2. In `FFXIV.cfg` (`~/.local/share/dev.goats.xivlauncher/ffxivConfig/`), under `<GamePad Settings>`, delete the
   `InstanceGuid`, `ProductGuid` and `Alias` lines.

The game then builds its default DualSense layout and saves the three entries back as zeros. Repeat step 1 after
any session that put the pad in XInput mode.

With Proton logging on in XIVLauncher, `WINEDEBUG` output goes to `logs/steam-default.log`, not `logs/wine.log`.

Audio endpoint names, for reference: FFXIV is reported to pick the haptic endpoint by a name containing "Wireless
Controller" ([Proton#5900](https://github.com/ValveSoftware/Proton/issues/5900)). Valve's winepulse uses the sink
description when it is 62 characters or fewer; "DualSense wireless controller (PS5) Direct Wireless Controller" is
exactly 62. GE-Proton's patch 0174 names Direct-profile endpoints "DualSense wireless controller (PS5) Internal Mono
Speaker".
