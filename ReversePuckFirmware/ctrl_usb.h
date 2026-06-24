// ctrl_usb.h -- the emulated controller's USB face (Valve 28DE:1302) + pairing command channel.
//
// Presents a single Valve HID control interface (usagePage 0x01, feature reports id 1/2) so Steam and
// PairTUI treat us as a dockable Steam Controller. We ANSWER identity reads (0x83 product, 0xAE serial,
// 0xB4 wireless state) and ACCEPT the steamless pairing writes that target the controller:
//   0xEE  keyed value write   payload ["esb/bond"\0 or "esb/bond_2"\0][r1 4][r2 4][puck_serial 16]
//   0xEF  keyed value commit  payload ["esb/bond"\0]
//   0xA2  direct 24-byte bond write to slot 0 (PairTUI/Steam fallback shape)
//   0x95  reboot into wireless mode  (magic 0xA427AF52) -> NVIC_SystemReset after the flash flush
// See protocol/USB_COMMANDS.md for the full steamless flow.
#pragma once
#include <stdint.h>

// register the HID interface (call from setup, before USBDevice.attach)
void ctrlUsbBegin();
// flush dirty bonds + honor a pending reboot (call each loop)
void ctrlUsbPoll();

// Build the controller's reply to a feature/command GET (0x83 attrs, 0xAE serial, 0xB4 wireless state)
// as [cmd][len][payload] -- the command-channel form a real controller returns over USB (WITH the inner
// length byte). ctrl_link uses this to answer the queries the puck relays over RF as E3 GETs (Steam's
// enumeration). `param`/`plen` are the GET parameters (e.g. the 0xAE index). Returns out length.
uint8_t ctrlFeatureResp(uint8_t cmd, const uint8_t *param, uint8_t plen,
			uint8_t *out);
