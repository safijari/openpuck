// WebUSB configuration channel: a vendor interface present in EVERY mode so the browser config panel can
// read/set runtime tunables. Binary framed protocol:
//   host->dev:  0x01                 GET  -> reply status blob
//               0x02 <field> <value> SET one byte field (see the switch in webusb_config.cpp)
//               0x03 <mode>          switch USB mode (0..6): persist + reboot
//   dev->host:  0xA5 <len> <payload>  status blob (mode/tunables/link state/rates)
// No setLandingPage() on purpose -- it would pop a Chrome "open <url>?" notification on every plug-in.
#pragma once
#include <Adafruit_TinyUSB.h>

extern Adafruit_USBD_WebUSB usb_web;

void webusbPoll(); // drain host commands + reply; call every loop()
