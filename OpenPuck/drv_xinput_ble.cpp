#include "drv_xinput_ble.h"
#include "triton.h"
#include <string.h>
#include <ctype.h>

XInputBleDriver g_drvXInputBle;

// newlib has no strcasestr; tiny local case-insensitive substring match
static bool nameHas(const char *hay, const char *needle)
{
	size_t nl = strlen(needle);
	for (; *hay; hay++) {
		size_t i = 0;
		while (i < nl && hay[i] &&
		       tolower((unsigned char)hay[i]) ==
			       tolower((unsigned char)needle[i]))
			i++;
		if (i == nl)
			return true;
	}
	return false;
}

// Microsoft's VID (real Xbox pads AND the 8BitDo xinput-mode spoof) plus 8BitDo's own VID, in case a future
// 8BitDo BLE firmware advertises the honest id. Name fallback covers pads whose DIS/PnP ID read fails (the
// Xbox pads expose it, but only after encryption -- ble_host reads it post-pairing, so this is a backstop).
bool XInputBleDriver::match(uint16_t vid, uint16_t pid,
			    const char *devName) const
{
	(void)pid; // all Microsoft gamepad PIDs (02E0/02FD/0B13/...) share the layout
	if (vid == 0x045E || vid == 0x2DC8)
		return true;
	return nameHas(devName, "xbox") || nameHas(devName, "8bitdo");
}

// HID service report characteristics in declaration order: [0] = input 0x01 (the gamepad state; fw 5.x pads
// carry guide/share in it too), [1] = input 0x02 (the legacy separate guide-button report; pre-fw5 / 8BitDo).
// The output (rumble) characteristic is not in this mapping -- it is identified by having no CCCD and always
// carries report 0x03 on these pads.
uint8_t XInputBleDriver::inputRidByIndex(uint8_t idx) const
{
	return (uint8_t)(idx + 1); // 0 -> 0x01, 1 -> 0x02
}

// 8-way hat -> TB_ dpad bits. 0 = released, 1..8 = N NE E SE S SW W NW.
static uint32_t hatBits(uint8_t h)
{
	static const uint32_t T[9] = {
		0,
		TB_DUP,
		TB_DUP | TB_DRT,
		TB_DRT,
		TB_DRT | TB_DDN,
		TB_DDN,
		TB_DDN | TB_DLF,
		TB_DLF,
		TB_DLF | TB_DUP,
	};
	return h <= 8 ? T[h] : 0;
}

// Input report 0x01 (Xbox One S / Series BT+BLE layout, 14..16 bytes):
//   [0..7]  LX LY RX RY  u16 LE, 0..65535, center 32768, HID Y grows DOWN
//   [8..11] LT RT        u16 LE, 10-bit (0..1023)
//   [12]    hat          0=idle, 1..8 clockwise from N
//   [13]    A B X Y LB RB View Menu
//   [14]    (fw 5.x) bit0 Xbox/guide, bit1 LS, bit2 RS   (pre-fw5: bit0 LS, bit1 RS; guide rides report 0x02)
//   [15]    (Series 1914) bit0 Share
// Report 0x02: [0] bit0 = Xbox/guide (the pre-fw5 / 8BitDo location).
bool XInputBleDriver::decode(int slot, uint8_t rid, const uint8_t *d,
			     uint16_t len, PuckInput *io)
{
	(void)slot;
	if (rid == 0x02) {
		if (len < 1)
			return false;
		if (d[0] & 0x01)
			io->buttons |= TB_STEAM;
		else
			io->buttons &= ~TB_STEAM;
		return true;
	}
	if (rid != 0x01 || len < 14)
		return false;
	uint16_t lx = (uint16_t)(d[0] | (d[1] << 8)),
		 ly = (uint16_t)(d[2] | (d[3] << 8));
	uint16_t rx = (uint16_t)(d[4] | (d[5] << 8)),
		 ry = (uint16_t)(d[6] | (d[7] << 8));
	uint16_t lt = (uint16_t)(d[8] | (d[9] << 8)),
		 rt = (uint16_t)(d[10] | (d[11] << 8));
	uint8_t b1 = d[13];
	uint8_t b2 = (len > 14) ? d[14] : 0;
	uint8_t b3 = (len > 15) ? d[15] : 0;

	uint32_t b = hatBits(d[12]);
	if (b1 & 0x01)
		b |= TB_A;
	if (b1 & 0x02)
		b |= TB_B;
	if (b1 & 0x04)
		b |= TB_X;
	if (b1 & 0x08)
		b |= TB_Y;
	if (b1 & 0x10)
		b |= TB_LB;
	if (b1 & 0x20)
		b |= TB_RB;
	if (b1 & 0x40)
		b |= TB_VIEW;
	if (b1 & 0x80)
		b |= TB_MENU;
	if (len > 14) {
		if (b2 & 0x01)
			b |= TB_STEAM; // fw 5.x: guide is in-band
		if (b2 & 0x02)
			b |= TB_L3;
		if (b2 & 0x04)
			b |= TB_R3;
	} else {
		// 14-byte variant: guide arrives on report 0x02 -- preserve its latched state
		b |= io->buttons & TB_STEAM;
	}
	if (b3 & 0x01)
		b |= TB_QAM; // Series share button -> QAM (3 dots)
	// analog triggers 0..1023 -> 0..255, full-pull digital bits like the SC2 sets them
	if (lt >= 1000)
		b |= TB_L2;
	if (rt >= 1000)
		b |= TB_R2;

	io->buttons = b;
	io->lx = (int16_t)(lx - 32768);
	io->ly = (int16_t)(32767 - ly); // HID Y down -> gamepad Y up
	io->rx = (int16_t)(rx - 32768);
	io->ry = (int16_t)(32767 - ry);
	io->lt = (uint8_t)(lt >> 2 > 255 ? 255 : lt >> 2);
	io->rt = (uint8_t)(rt >> 2 > 255 ? 255 : rt >> 2);
	// no trackpads / IMU on this device class: leave the zeroed fields alone
	return true;
}

// Rumble output report 0x03 (hid-microsoft xb1s_ff_report): [enable][magLT][magRT][magStrong][magWeak]
// [sustain 10ms][release 10ms][loop]. Magnitudes 0..100. Enable all four motors -- the trigger magnitudes stay
// 0, so pads without impulse triggers just ignore those bits. Continuous-stream semantics like every other
// rumble sink here: the host re-sends on change, 0/0 stops.
uint8_t XInputBleDriver::buildRumble(uint16_t lo, uint16_t hi, uint8_t out[16])
{
	out[0] = 0x0F;
	out[1] = 0;
	out[2] = 0;
	out[3] = (uint8_t)((uint32_t)lo * 100 / 0xFFFF);
	out[4] = (uint8_t)((uint32_t)hi * 100 / 0xFFFF);
	out[5] = 0xFF; // sustain ~2.5s; the stream refreshes long rumbles
	out[6] = 0x00;
	out[7] = 0x00; // no repeat -- a lost "off" must not buzz forever
	return 8;
}
