# Build And Deploy

These instructions cover firmware builds for macOS, Linux, and Windows, plus static hosting of the WebUSB app through GitHub Pages.

## 1. Prerequisites

Hardware:

- nRF52840 board supported by the Adafruit nRF52 Arduino core
- USB cable
- Steam Controller 2 controller

Software:

- `arduino-cli`
- Adafruit nRF52 Arduino core
- Chrome or Edge for the WebUSB app

## 2. Install Arduino CLI

### macOS

```bash
brew install arduino-cli
```

### Linux

Use your package manager if it ships a recent `arduino-cli`, or download the release archive from Arduino and place `arduino-cli` on `PATH`.

### Windows (CLI)

Install the official Arduino CLI zip or MSI, then ensure `arduino-cli.exe` is on `PATH`.

## 3. Install the board core

Run once on any platform:

```bash
arduino-cli config init
arduino-cli core update-index
arduino-cli core install adafruit:nrf52
```

If your CLI config does not already include the Adafruit package index, add it first:

```yaml
board_manager:
  additional_urls:
    - https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
```

Then rerun `arduino-cli core update-index`.

If uing the GUI, go to File -> Preferences and paste the board manager URL above into the "Additional Boards Manager URLs" dialog.
The go to Tools -> Boards -> Boards Manager, search for nrf and install the Adafruit nRF52 module.

## 4. Build the firmware

From the repository root:

```bash
arduino-cli compile \
  -b adafruit:nrf52:feather52840 \
  --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_HID=4" \
  OpenPuck
```

This sketch requires `CFG_TUD_HID=4` because Steam mode exposes four HID interfaces.

## 5. Upload the firmware

### macOS / Linux

Find the board port:

```bash
arduino-cli board list
```

Upload:

```bash
arduino-cli upload \
  -b adafruit:nrf52:feather52840 \
  -p /dev/ttyACM0 \
  OpenPuck
```

Replace `/dev/ttyACM0` with the actual port. On macOS it is usually `/dev/cu.usbmodem*`.

### Windows

Find the COM port:

```powershell
arduino-cli board list
```

Upload:

```powershell
arduino-cli upload `
  -b adafruit:nrf52:feather52840 `
  -p COM5 `
  OpenPuck
```

Replace `COM5` with the actual board port.

## 6. Pair and verify

1. Flash the board.
2. Plug it into the host.
3. In Steam mode it enumerates as a puck-compatible device.
4. Pair the controller to one of the bond slots.
5. Verify that the slot returns `0xB4 = 0x02` when connected.

## 7. Run the WebUSB app locally

WebUSB requires a secure context. `http://localhost` qualifies.

### macOS / Linux

```bash
cd docs
python3 -m http.server 8008
```

Open:

```text
http://localhost:8008
```

### Windows

```powershell
cd docs
py -m http.server 8008
```

Open:

```text
http://localhost:8008
```

## 8. Known operational details

- Chrome or Edge is required for WebUSB.
- The board re-enumerates on USB mode switches.
- Poll interval tuning in the WebUSB app is session-only by design.
- Bonds persist in the board's internal filesystem.
