# ReversePuck — Steam Deck forwarder

Turn a Steam Deck into a Steam Controller 2. The Deck's built-in controls are **grabbed** (detached) and
streamed over USB-CDC to a **ReversePuck** nRF52840 (firmware in
[`../ReversePuckFirmware/`](../ReversePuckFirmware)), which relays them over 2.4 GHz to a puck — a real
Valve puck **or** an [OpenPuck](../OpenPuck) — so the puck presents a Steam Controller 2 to whatever host
it's plugged into.

```
Deck controls ──libusb detach + raw DeckState──▶ openctrl.py ──USB-CDC──▶ nRF (28DE:1302) ──RF──▶ puck ──USB──▶ host
```

The nRF "dongle" (the ReversePuck firmware, USB `28DE:1302`) is **hot-pluggable**: launch the app with or
without it plugged in, and it appears/disappears live. Unplug it mid-forward and control returns to the
Deck automatically; plug it back and the tile reappears — no restart needed.

## Input: USB-level detach (the right way)
The Deck controller is USB `28DE:1205`; Steam reads its 64-byte "DeckState" report on interface 2.
You **can't** block Steam by reading `/dev/hidraw3` (it's broadcast), so `openctrl` uses **libusb
(pyusb)** to **detach the kernel driver from the controller's HID interfaces and read EP `0x83`
directly** — that genuinely takes the pad away from Steam (so **Steam/QAM forward** instead of opening
the Deck UI) and decodes the **raw report correctly** (trackpads, sticks, triggers, gyro, all buttons —
no more trackpad→stick/dpad bleed or Start/Select swap). Stopping forwarding reattaches it to Steam.
Modeled on `~/Work/joverlay`. (`--evdev` is a lower-fidelity fallback that can't detach at the USB level.)

**Calibrate the report layout for your unit:** `sudo python3 input_source.py` live-dumps the raw
DeckState and shows which bytes/bits change as you press each control — edit `DECK_BTN_MAP` / the
analog offsets in `input_source.py` if anything's off. The IMU axis remap (`IMU_GYRO_MAP` /
`IMU_ACCEL_MAP`) is calibrated in Steam's gyro test — see the comments above those constants.

## Pieces

| File | What |
|---|---|
| `openctrl.py` | Main app: opens the nRF CDC port, shows the touchscreen UI, forwards input. |
| `ui.py` | Fullscreen pygame UI — a tappable tile per paired puck. |
| `input_source.py` | Reads the Deck controls — **libusb DeckSource (detach + raw decode)** + evdev fallback + capture tool. |
| `frame.py` | The USB-CDC wire protocol (kept in lock-step with `../ReversePuckFirmware/deck_input.*`). |
| `scpair.py` | Linux hidraw pairing tool (the on-Deck equivalent of `pairtui`). |
| `ReversePuck` | `uv` launcher (`pyserial`+`pygame`+`pyusb`) — add this as a non-Steam game. |
| `setup.sh` | Per-boot `sudo`: opens `/dev/ttyACM*` + the controller's usbfs node (read-only SteamOS). |

## Run with `uv` (no system install, read-only SteamOS friendly)

The `ReversePuck` launcher runs the app via [`uv`](https://docs.astral.sh/uv/), which provisions its own
Python and the three deps (`pyserial` + `pygame` + `pyusb`, all prebuilt wheels; `pyusb` uses the system
`libusb-1.0.so`) into uv's cache — **nothing touches the read-only rootfs**. The launcher **auto-installs
`uv` on first run** if it isn't already present (into `~/.local/bin`, no root), so the manual step below is
only needed if you'd rather install it yourself or the auto-install can't reach the network.

```bash
# optional: the launcher does this for you on first run (lands in ~/.local/bin, no root)
curl -LsSf https://astral.sh/uv/install.sh | sh

./ReversePuck            # fullscreen touchscreen UI (first run pulls deps)
./ReversePuck --debug    # headless status print (bring-up)
./ReversePuck --evdev    # evdev fallback: no USB detach (Steam/QAM won't forward, lower-fidelity pads)
```

### Add as a non-Steam game
Add the `ReversePuck` launcher as a non-Steam game (Target = `ReversePuck`, **Start In** = this folder).
In the shortcut's **Properties → Controller**, set **Disable Steam Input** so Steam doesn't fight us for
the built-in pad while we grab it.

### Device permissions (read-only rootfs, no persistent udev)
`/dev/input/event*` is already granted to the active-session user (logind ACL), so the `--evdev` fallback
works without any setup. The two devices the **default** (libusb) path needs are not, so run once per
boot from Desktop Mode:

```bash
sudo ./setup.sh     # chmod a+rw /dev/ttyACM* (nRF CDC) + the Deck controller's usbfs node; not persistent
```

`setup.sh` opens the nRF CDC (`/dev/ttyACM*`) and the Deck controller's `/dev/bus/usb/...` node (so the
libusb `DeckSource` can detach + read it). We can't install a udev rule because `/etc` is read-only.
Without it you'll get a permission error and the app falls back to `--evdev`.

## Pair the controller to a puck

Pairing writes a shared key to **both** devices, so both must be on USB **at the same time**, on one
computer. Two ways:

- **Steam / PairTUI (any Mac/PC):** plug in the nRF (enumerates as Valve `28DE:1302`, a "Steam
  Controller") and the puck (`28DE:1304`), then pair as usual — `../pairtui/pairtui.py` already treats
  `1302` as a dockable controller and `1304` as the puck.
- **On the Deck (Linux):** plug both into the Deck and run
  ```bash
  sudo python3 scpair.py list           # show puck slots + controllers
  sudo python3 scpair.py pair           # pair into the first free slot
  sudo python3 scpair.py unpair --slot 0
  ```
  `pair` mints a fresh key, writes the puck slot (`0xA2`) and the controller bond (`esb/bond` via
  `0xEE`/`0xEF`), then reboots the controller into wireless (`0x95`). Move the puck to your host.

### "Steam says pairing failed" (but it actually works)
Steam's pairing IS the USB bond-write path above (`0xA2`/`0xEE`/`0xEF` then a `0x95` reboot-to-wireless),
verified end-to-end. If Steam's dialog reports failure even though the puck connects afterwards, the
suspect is Steam's post-`0x95` confirmation racing the controller's reboot (the reboot drops USB).

The firmware now **defers the `0x95` reset until the host has read the command's ACK back** (or a short
grace window lapses), so the USB drop no longer lands mid-transaction — see
`../ReversePuckFirmware/ctrl_usb.cpp` (`g_respRead` / `REBOOT_GRACE_MS`).

To pin down exactly what Steam polls before declaring failure, watch the firmware's serial monitor during
a pairing attempt: it logs every feature-report `SET` **and** `GET` (`# SET …` / `# GET rid=… -> …`). The
last command Steam issues before the dialog fails is the one to reproduce. (The Deck app's framed protocol
resyncs past these `#` lines, so they're harmless when forwarding.)

## Using it

Tap a puck tile that shows **● available** to detach the Deck controls and start forwarding. Tap
anywhere (or unplug the puck/dongle) to hand control back to the Deck. `Esc`/`Q` quits. The app survives
the dongle or the Deck pad being unplugged and replugged mid-session — it tears down cleanly and rebuilds
when the device reappears, so a transient USB hiccup never wedges it or leaves the pad detached.

Validate the RF link first with `./ReversePuck --debug` (headless): you want a line like
`link=True fwd=False ch=18 [0 FXB99602xxxxx LIVE]`. Keep the puck on USB (so it beacons) and the nRF
plugged in.

## Manage & flash the dongle from the browser panel

The nRF dongle firmware (`../ReversePuckFirmware`) now exposes a WebUSB vendor interface, so the OpenPuck
browser panel (`docs/index.html`, hosted at the project's GitHub Pages) connects to it just like it does a
puck. Plug the dongle into any Chrome/Edge machine, click **Connect**, and pick the `28DE:1302` device. The
panel detects the controller and shows a reduced UI:

- **Paired pucks** — every puck the dongle is bonded to, with a live/offline badge and a **Remove**
  (un-bond) button. Same list the Deck app shows over CDC, just over WebUSB.
- **Firmware update** — drop a `ReversePuckFirmware` `.uf2` on the card to stream it over WebUSB (verified
  on-device, applied on an automatic reboot), or use **UF2 DFU** / **Serial DFU** in the top bar to reboot
  into the bootloader for drag-and-drop / `adafruit-nrfutil` flashing.

Build a flashable dongle image with `make reversepuck` (from the repo root); flash a connected dongle with
`make reversepuck-deploy <port>` (double-tap reset for the bootloader first — see `docs/BUILD_AND_DEPLOY.md`).

## Notes / known iteration points

- **IMU (gyro + accel)** is read straight from the libusb `DeckSource` and remapped to the SC2 axes via
  `IMU_GYRO_MAP` / `IMU_ACCEL_MAP` in `input_source.py`. Accel and gyro must share handedness or the
  host's sensor fusion mixes axes — calibrate both together in Steam's gyro test.
- **Game Mode / gamescope** — the libusb detach and evdev `EVIOCGRAB` both behave cleanly in **Desktop
  Mode**. In Game Mode, Steam Input owns the controller, so detach behaves differently; run from Desktop
  Mode for now.
- **Button mapping** lives in `input_source.DECK_BTN_MAP` (libusb) / `KEYMAP` (evdev); adjust if your
  Deck maps face buttons or back paddles differently.
- **Real Valve puck interop** — the RF reconnect (`../ReversePuckFirmware/ctrl_link.cpp`) adopts the
  session address the puck advertises in its `E1` host frame; this matches OpenPuck exactly and should
  match a real puck (OpenPuck reproduces the real puck's `E1`). If a real puck won't connect, the `E1`
  field offsets are the first thing to check on a capture.
