# OpenController — Steam Deck forwarder

Turn a Steam Deck into a Steam Controller 2. The Deck's built-in controls are **grabbed** (detached) and
streamed over USB-CDC to an **OpenController** nRF52840 (firmware in [`../OpenController/`](../OpenController)),
which relays them over 2.4 GHz to a puck — a real Valve puck **or** an [OpenPuck](../OpenPuck) — so the
puck presents a Steam Controller 2 to whatever host it's plugged into.

```
Deck controls ──libusb detach + raw DeckState──▶ openctrl.py ──USB-CDC──▶ nRF (28DE:1302) ──RF──▶ puck ──USB──▶ host
```

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
analog offsets in `input_source.py` if anything's off.

## Pieces

| File | What |
|---|---|
| `openctrl.py` | Main app: opens the nRF CDC port, shows the touchscreen UI, forwards input. |
| `ui.py` | Fullscreen pygame UI — a tappable tile per paired puck. |
| `input_source.py` | Reads the Deck controls — **libusb DeckSource (detach + raw decode)** + evdev fallback + capture tool. |
| `frame.py` | The USB-CDC wire protocol (kept in lock-step with `../OpenController/deck_input.*`). |
| `scpair.py` | Linux hidraw pairing tool (the on-Deck equivalent of `pairtui`). |
| `run.sh` | `uv` launcher (`pyserial`+`pygame`+`pyusb`) — add this as a non-Steam game. |
| `setup.sh` | Per-boot `sudo`: opens `/dev/ttyACM*` + the controller's usbfs node (read-only SteamOS). |

## Run with `uv` (no system install, read-only SteamOS friendly)

`run.sh` launches the app via [`uv`](https://docs.astral.sh/uv/), which provisions its own Python and
the two deps (`pyserial` + `pygame`, both prebuilt wheels) into uv's cache — **nothing touches the
read-only rootfs**. Input is read with pure-stdlib evdev ioctls, so there's no `evdev` C-extension to
compile.

```bash
# one-time: install uv (lands in ~/.local/bin, no root)
curl -LsSf https://astral.sh/uv/install.sh | sh

./run.sh            # fullscreen touchscreen UI (first run pulls deps)
./run.sh --debug    # headless status print (bring-up)
./run.sh --hidraw   # also forward gyro + trackpads (offsets unverified, see notes)
```

### Add as a non-Steam game
Add `run.sh` as a non-Steam game (Target = `run.sh`, **Start In** = this folder). In the shortcut's
**Properties → Controller**, set **Disable Steam Input** so Steam doesn't fight us for the built-in pad.

### Device permissions (read-only rootfs, no persistent udev)
`/dev/input/event*` is already granted to the active-session user (logind ACL), so the evdev grab works
without any setup. The nRF CDC (`/dev/ttyACM*`) is not, so if you get a serial permission error, run
once per boot from Desktop Mode:

```bash
sudo ./setup.sh     # chmod a+rw /dev/ttyACM* (+ /dev/hidraw* for --hidraw); not persistent
```

(We can't install a udev rule because `/etc` is read-only and you can't disable that.)

## Pair the controller to a puck

Pairing writes a shared key to **both** devices, so both must be on USB **at the same time**, on one
computer. Two ways:

- **Steam / PairTUI (any Mac/PC):** plug in the nRF (enumerates as Valve `28DE:1302`, a "Steam
  Controller") and the puck (`28DE:1304`), then pair as usual — `../pairtui/pairtui.py` already treats
  `1302` as a dockable controller and `1304` as the puck.
- **On the Deck (Linux):** plug both into the Deck and run
  ```bash
  python3 scpair.py list          # show puck slots + controllers
  python3 scpair.py pair           # pair into the first free slot
  python3 scpair.py unpair --slot 0
  ```
  `pair` mints a fresh key, writes the puck slot (`0xA2`) and the controller bond (`esb/bond` via
  `0xEE`/`0xEF`), then reboots the controller into wireless (`0x95`). Move the puck to your host.

## Using it

Tap a puck tile that shows **● available** to detach the Deck controls and start forwarding. Tap
anywhere (or unplug the puck) to hand control back to the Deck. `Esc`/`Q` quits.

Validate the RF link first with `./run.sh --debug` (headless): you want a line like
`link=True fwd=False ch=18 [0 FXB99602xxxxx LIVE]`. Keep the puck on USB (so it beacons) and the nRF
plugged in.

## Notes / known iteration points

- **Gyro + trackpads** come from the Deck's `hid-steam` hidraw report (`--hidraw`). The report offsets
  in `input_source.HidrawImu` are a documented starting point and **must be confirmed against a real
  Deck capture** — until then forwarding works with buttons/sticks/triggers (evdev) and zero IMU.
- **Game Mode / gamescope** — evdev `EVIOCGRAB` detaches cleanly in **Desktop Mode**. In Game Mode,
  Steam Input owns the controller, so detach behaves differently; run from Desktop Mode for now.
- **Button mapping** lives in `input_source.KEYMAP` / `EvdevSource._apply`; adjust if your Deck maps
  face buttons or back paddles differently.
- **Real Valve puck interop** — the RF reconnect (`../OpenController/ctrl_link.cpp`) adopts the session
  address the puck advertises in its `E1` host frame; this matches OpenPuck exactly and should match a
  real puck (OpenPuck reproduces the real puck's `E1`). If a real puck won't connect, the `E1` field
  offsets are the first thing to check on a capture.
