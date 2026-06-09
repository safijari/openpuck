// bonds.h -- the puck's bond slots + live link state.
//
// The real puck holds four controller bonds and exposes one HID control interface per slot (interface N owns
// slot N). We mirror that: g_slot[N] holds the bond record [8 uuid][16 serial] plus a staging buffer for that
// interface's pending feature-report reply (filled by puck_hid.cpp's command channel). Bonds persist to
// flash (bonds.bin). g_connSlot / g_connReplyMs are the live RF link state, shared here so the USB feature
// handler can report per-slot connection status without depending on the RF layer.
#pragma once
#include <stdint.h>

#define NSLOT 4

// per-slot state (record = [8 uuid][16 serial]) + per-interface response staging
struct Slot { uint8_t rec[24]; bool used; uint8_t resp[63]; uint16_t resp_len; };
extern Slot g_slot[NSLOT];

extern int           g_connSlot;     // bonded slot being polled (== the USB-connected slot); -1 = none
extern unsigned long g_connReplyMs;  // millis of last RF reply (link-alive timestamp)
extern volatile bool g_dirty;        // bonds changed -> flush to flash from loop()
extern bool          g_pairing;

bool recEmpty(const uint8_t *r);
void loadBonds();
void saveBonds();
