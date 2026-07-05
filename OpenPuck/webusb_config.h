// WebUSB configuration channel: a vendor interface present in EVERY mode so the browser config panel can
// read/set runtime tunables. Binary framed protocol:
//   host->dev:  0x01                 GET  -> reply status blob
//               0x02 <field> <value> SET one byte field (see the switch in webusb_config.cpp)
//               0x03 <mode>          switch USB mode (0..6): persist + reboot
//               0x0B                 reboot into serial DFU (adafruit-nrfutil)
//               0x0C                 reboot into UF2 bootloader (USB mass storage)
//               0x20 <size u32> <crc32 u32>        firmware update: begin staging (disarms any previous)
//               0x21 <off u32> <len> <data...>     firmware update: sequential chunk (len<=128, %4==0)
//               0x22                 firmware update: verify staged image + COMMIT (applies on next reboot)
//               0x23                 clean reboot (after 0x22: boots into the freshly staged firmware)
//               0x24                 firmware update: abort/disarm
//   dev->host:  0xA5 <len> <payload>  status blob (mode/tunables/link state/rates)
//               0xAB 5 <status> <nextOff u32>  firmware-update ack, one per 0x20/0x21/0x22/0x24 (fw_update.h)
// No setLandingPage() on purpose -- it would pop a Chrome "open <url>?" notification on every plug-in.
#pragma once
#include <Adafruit_TinyUSB.h>

extern Adafruit_USBD_WebUSB usb_web;

void webusbInit(); // register the SOF-driven blob sender; call once from setup() after usbTxBegin()
void webusbPoll(); // drain host commands + reply; call every loop()
