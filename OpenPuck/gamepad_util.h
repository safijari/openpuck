// gamepad_util.h -- shared helpers for building host USB gamepad reports from g_in.
//
// These are reused across the DualSense / DS4 / Switch personalities: analog-stick rescaling, Steam-trackpad
// -> absolute-touch mapping, and the PlayStation-layout button/hat/face/shoulder packers. Per-target button
// code maps (codeToXB / codeToSwitch / codeToJc) live with their own controllers; only the cross-mode pieces
// are here.
#pragma once
#include <stdint.h>

// int16 stick (center 0) -> uint8 (center 0x80), optional axis invert (HID Y is down-positive).
uint8_t swStick(int16_t v, bool invert);

// Steam trackpad s16 coords -> absolute touch surface. TOUCH_PAD_W is split into left/right halves so both
// pads can co-exist as two contacts on a single DualSense/DS4 touchpad.
#define TOUCH_PAD_W 1920u
uint16_t padNormU16(int16_t v, uint16_t maxv);
uint16_t touchHalfX(int16_t v, bool rightHalf);
uint16_t touchYInv(int16_t v, uint16_t height);
void touchPackPoint(uint8_t* base, int finger, bool touch, uint16_t x, uint16_t y);
// DualSense / DS4: many hosts only honor contact slot 0 -- put a lone pad in slot 0, both pads in 0+1.
void touchPackPads(uint8_t* pts, bool lTouch, bool rTouch, uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry);
void steamPadsToTouch(uint32_t b, uint16_t touchH, int16_t lpx, int16_t lpy, int16_t rpx, int16_t rpy,
    uint16_t* lx, uint16_t* ly, uint16_t* rx, uint16_t* ry);

// PlayStation-layout button packing (shared by mode_ps5 + mode_hidgyro).
uint32_t psButtonsFromSteam(uint32_t raw);   // apply back-paddle + chord-guard remapping
uint8_t  psShouldersByte(uint32_t b);        // L1..R3 + Create/Options byte
uint8_t  psHatNibble(uint32_t b);            // d-pad -> 8-way hat (8 = neutral)
uint8_t  psFaceNibble(uint32_t b);           // face buttons (with A/B + X/Y swap)
