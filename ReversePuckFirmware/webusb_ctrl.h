// webusb_ctrl.h -- the ReversePuck dongle's WebUSB configuration channel (28DE:1302).
//
// Adds a vendor interface so the OpenPuck browser panel (docs/index.html) can connect to the CONTROLLER
// dongle -- distinct from the puck's own panel (28DE:1304). It exposes a SUBSET of the puck's WebUSB
// protocol (webusb_config.h), reusing the SAME firmware-update opcodes + 0xAB ack framing so the panel's
// existing streaming flasher and DFU buttons work against the dongle unchanged. The panel branches on the
// USB product id (0x1302 vs 0x1304) to show the reduced controller UI.
//
// Binary framed protocol:
//   host->dev:  0x01                 GET  -> reply the paired-pucks list (0xAC frame)
//               0x0B                 reboot into serial DFU (adafruit-nrfutil)
//               0x0C                 reboot into UF2 bootloader (USB mass storage)
//               0x20 <size u32> <crc32 u32>     firmware update: begin staging (disarms any previous)
//               0x21 <off u32> <len> <data...>  firmware update: sequential chunk (len<=128, %4==0)
//               0x22                 firmware update: verify staged image + COMMIT (applies on next reboot)
//               0x23                 clean reboot (after 0x22: boots the freshly staged firmware)
//               0x24                 firmware update: abort/disarm
//               0x25 "WIPE"          FULL BOARD WIPE (debug): erase app+bonds+bl-settings, reboot app-less
//               0x30 <slot>          un-bond (remove) one paired puck by firmware slot, then reply 0xAC
//   dev->host:  0xAC <len> <ver=1> <flags> <count> [ per bond: slot, alive, puuid4, iuuid4, serial16 (26B) ]
//                                    paired-pucks list (flags bit0=forwarding, bit1=link up)
//               0xAB 5 <status> <nextOff u32>  firmware-update ack, one per 0x20/0x21/0x22/0x24 (fw_update.h)
// No setLandingPage() on purpose -- it would pop a Chrome "open <url>?" prompt on every plug-in.
#pragma once
#include <Adafruit_TinyUSB.h>

extern Adafruit_USBD_WebUSB usb_web;

void webusbCtrlBegin(); // add the vendor interface; call in setup() BEFORE USBDevice.attach()
void webusbCtrlPoll(); // drain host commands + flush replies; call every loop()
