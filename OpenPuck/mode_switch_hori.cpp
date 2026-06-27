#include "mode_switch_hori.h"
#include "triton.h"
#include "gamepad_util.h"
#include "config.h"
#include "bonds.h"
#include "usb_mount.h"
#include "usb_tx.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

SwitchHoriController g_switchHori;

// 8-byte input report: [btn_lo][btn_hi][hat][LX][LY][RX][RY][vendor], sticks uint8 center 0x80.
static const uint8_t SWITCH_HID_DESC[] = {

	// Usage Page (Generic Desktop), Usage (Game Pad), Collection (App)
	0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0x15, 0x00, 0x25, 0x01, 0x35, 0x00,
	0x45, 0x01, 0x75, 0x01, 0x95, 0x10, // 16 x 1-bit buttons
	0x05, 0x09, 0x19, 0x01, 0x29, 0x10, 0x81, 0x02, 0x05, 0x01, 0x25, 0x07,
	0x46, 0x3B, 0x01, 0x75, 0x04, 0x95, 0x01, 0x65, 0x14, 0x09, 0x39, 0x81,
	0x42, // hat switch (4 bits)
	0x65, 0x00, 0x95, 0x01, 0x81, 0x01, // 4-bit padding to a byte
	0x26, 0xFF, 0x00, 0x46, 0xFF, 0x00, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32,
	0x09, 0x35, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, // X,Y,Z,Rz

	// vendor byte (input)
	0x06, 0x00, 0xFF, 0x09, 0x20, 0x95, 0x01, 0x81, 0x02,

	// vendor (output, 8 bytes; unused)
	0x0A, 0x21, 0x26, 0x95, 0x08, 0x91, 0x02, 0xC0
};
// NSLOT HORIPAD HID instances -- one per bond slot. The Switch console binds each as a separate gamepad.
static Adafruit_USBD_HID g_switch[NSLOT];
static unsigned long g_swLastMs[NSLOT] = { 0 };

// back-paddle / QAM code (g_back[], g_qamMap) -> Switch button bit. 0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back(Minus) 10=Start(Plus) 11=Guide(Home) 18=Capture
static uint16_t codeToSwitch(uint8_t c, uint16_t fA, uint16_t fB, uint16_t fX,
			     uint16_t fY)
{
	switch (c) {
	case 1:
		return fA;
	case 2:
		return fB;
	case 3:
		return fX;
	case 4:
		return fY;
	case 5:
		return 0x10;
	case 6:
		return 0x20;
	case 7:
		return 0x400;
	case 8:
		return 0x800;
	case 9:
		return 0x100;
	case 10:
		return 0x200;
	case 11:
		return 0x1000;
	case 18:
		return 0x2000; // Capture / Screenshot (Switch-only target)
	case 19:
		return 0x40; // ZL (left trigger)
	case 20:
		return 0x80; // ZR (right trigger)
	default:
		return 0;
	}
}
// Back-paddle code 12..15 map to D-pad Up/Down/Left/Right; fold them into the hat direction flags.
static inline void backCodeToHatDirs(uint8_t c, bool &u, bool &d, bool &l,
				     bool &r)
{
	if (c == 12)
		u = true;
	else if (c == 13)
		d = true;
	else if (c == 14)
		l = true;
	else if (c == 15)
		r = true;
}
// HORIPAD/Switch button bits: Y=1 B=2 A=4 X=8 L=10 R=20 ZL=40 ZR=80 Minus=100 Plus=200 LClick=400 RClick=800 Home=1000 Capture=2000
// Per-slot: each controller's decoded input is in g_in[slot].
static void switchBuildHoripad(uint8_t slot, uint8_t out[8])
{
	uint32_t b = g_in[slot].buttons;
	uint16_t btn = 0;
	// QAM (3 dots) remap: the configured code is applied to the Switch output just like a back paddle below
	// (so Capture(18) and every Switch target work). 0 = unmapped -> QAM does nothing on Switch.
	bool qam = g_qamMap && (b & TB_QAM);
	// Mode-switch chord (all 4 back + A/X/Y): don't pass the face press to the console while the back-4 are held.
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_X | TB_Y);
	// face buttons with optional A/B + X/Y swap (Nintendo physical-vs-label layout)
	uint16_t fY = g_abSwap ? 0x08 : 0x01, fB = g_abSwap ? 0x04 : 0x02,
		 fA = g_abSwap ? 0x02 : 0x04, fX = g_abSwap ? 0x01 : 0x08;
	if (b & TB_Y)
		btn |= fY;
	if (b & TB_B)
		btn |= fB;
	if (b & TB_A)
		btn |= fA;
	if (b & TB_X)
		btn |= fX;
	if (b & TB_LB)
		btn |= 0x10;
	if (b & TB_RB)
		btn |= 0x20; // L, R
	// ZL/ZR digital: trip on the analog threshold (activates early) OR the full-press click bit
	if ((g_in[slot].lt >= SW_TRIG_ON) || (b & 0x8000000u))
		btn |= 0x40;
	if ((g_in[slot].rt >= SW_TRIG_ON) || (b & 0x800000u))
		btn |= 0x80;
	if (b & TB_MENU)
		btn |= 0x100;
	if (b & TB_VIEW)
		btn |= 0x200; // Minus, Plus
	if (b & TB_L3)
		btn |= 0x400;
	if (b & TB_R3)
		btn |= 0x800; // LClick, RClick
	if (b & TB_STEAM)
		btn |= 0x1000; // Home
	// back paddles -> configurable mapping (same g_back[] as Xbox: default L4->LB R4->RB L5->L3 R5->R3)
	if (b & TB_L4)
		btn |= codeToSwitch(g_back[0], fA, fB, fX, fY);
	if (b & TB_R4)
		btn |= codeToSwitch(g_back[1], fA, fB, fX, fY);
	if (b & TB_L5)
		btn |= codeToSwitch(g_back[2], fA, fB, fX, fY);
	if (b & TB_R5)
		btn |= codeToSwitch(g_back[3], fA, fB, fX, fY);
	if (qam)
		btn |= codeToSwitch(g_qamMap, fA, fB, fX, fY);
	bool u = b & TB_DUP, d = b & TB_DDN, l = b & TB_DLF,
	     r = b & TB_DRT; // hat: 0=N..7=NW, 8=neutral
	if (b & TB_L4)
		backCodeToHatDirs(g_back[0], u, d, l, r);
	if (b & TB_R4)
		backCodeToHatDirs(g_back[1], u, d, l, r);
	if (b & TB_L5)
		backCodeToHatDirs(g_back[2], u, d, l, r);
	if (b & TB_R5)
		backCodeToHatDirs(g_back[3], u, d, l, r);
	if (qam)
		backCodeToHatDirs(g_qamMap, u, d, l, r);
	uint8_t hat = 8;
	if (u && r)
		hat = 1;
	else if (r && d)
		hat = 3;
	else if (d && l)
		hat = 5;
	else if (l && u)
		hat = 7;
	else if (u)
		hat = 0;
	else if (r)
		hat = 2;
	else if (d)
		hat = 4;
	else if (l)
		hat = 6;
	out[0] = btn & 0xFF;
	out[1] = btn >> 8;
	out[2] = hat;
	out[3] = swStick(g_in[slot].lx, false);
	out[4] = swStick(g_in[slot].ly,
			 true); // HID Y is down-positive -> invert
	out[5] = swStick(g_in[slot].rx, false);
	out[6] = swStick(g_in[slot].ry, true);
	out[7] = 0;
}

// Dynamic-mount mode: begin() is unused (setup() calls beginPool()+usbReenumerate instead).
void SwitchHoriController::begin()
{
}
// Wake mouse (1 HID) is present in Switch mode, leaving CFG_TUD_HID-1 for the HORIPAD pool.
uint8_t SwitchHoriController::maxSlots() const
{
	uint8_t cap = (uint8_t)(CFG_TUD_HID - 1);
	return cap < NSLOT ? cap : (uint8_t)NSLOT;
}
void SwitchHoriController::usbIdentity()
{
	USBDevice.setID(0x0F0D, 0x0092);
	USBDevice.setDeviceVersion(0x0210);
	USBDevice.setManufacturerDescriptor("HORI CO.,LTD.");
	USBDevice.setProductDescriptor("POKKEN CONTROLLER");
}
void SwitchHoriController::beginPool()
{
	uint8_t pool = maxSlots();
	for (uint8_t s = 0; s < pool; s++) {
		g_switch[s].enableOutEndpoint(true);
		g_switch[s].setReportDescriptor(SWITCH_HID_DESC,
						sizeof SWITCH_HID_DESC);
		g_switch[s].setPollInterval(1);
		g_switch[s].begin();
	}
}
void SwitchHoriController::mountSlots(uint8_t k)
{
	for (uint8_t u = 0; u < k; u++)
		USBDevice.addInterface(g_switch[u]);
}
void SwitchHoriController::task()
{
	// stream the 8-byte HORIPAD report at ~250Hz per CONNECTED controller. usbSlot u -> bond slot for input;
	// switchBuildHoripad reads g_in[bond] (no per-USB-slot state in the HORIPAD report).
	for (uint8_t u = 0; u < g_usbMountCount; u++) {
		if (!g_switch[u].ready())
			continue;
		if (millis() - g_swLastMs[u] < USB_STREAM_MS)
			continue;
		int bond = g_usbToBond[u];
		if (bond < 0)
			continue;
		g_swLastMs[u] = millis();
		uint8_t p[8];
		switchBuildHoripad((uint8_t)bond, p);
		usbTxHid(&g_switch[u], 0, p,
			 sizeof p); // report-id-less descriptor
	}
}
