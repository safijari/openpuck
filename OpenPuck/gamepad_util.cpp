#include "gamepad_util.h"
#include "triton.h"
#include "config.h"

uint8_t swStick(int16_t v, bool invert)
{
	int32_t a = 0x80 + (invert ? -((int32_t)v >> 8) : ((int32_t)v >> 8));
	if (a < 0)
		a = 0;
	if (a > 255)
		a = 255;
	return (uint8_t)a;
}

// Map Steam trackpad s16 coords into a 0..max axis (centered touch -> mid-range).
uint16_t padNormU16(int16_t v, uint16_t maxv)
{
	int32_t t = ((int32_t)v + 32768);
	if (t < 0)
		t = 0;
	if (t > 65535)
		t = 65535;
	return (uint16_t)((t * (int32_t)maxv) / 65535);
}
uint16_t touchHalfX(int16_t v, bool rightHalf)
{
	uint16_t halfMax = TOUCH_PAD_W / 2 - 1;
	uint16_t x = padNormU16(v, halfMax);
	return rightHalf ? (uint16_t)(TOUCH_PAD_W / 2 + x) : x;
}
uint16_t touchYInv(int16_t v, uint16_t height)
{
	uint16_t maxy = height - 1;
	return (uint16_t)(maxy - padNormU16(v, maxy));
}
void touchPackPoint(uint8_t *base, int finger, bool touch, uint16_t x,
		    uint16_t y)
{
	uint8_t *f = base + finger * 4;
	if (!touch) {
		f[0] = 0x80;
		f[1] = 0;
		f[2] = 0;
		f[3] = 0;
		return;
	}
	f[0] = (uint8_t)(finger & 0x7F);
	f[1] = (uint8_t)(x & 0xFF);
	f[2] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
	f[3] = (uint8_t)((y >> 4) & 0xFF);
}
void touchPackPads(uint8_t *pts, bool lTouch, bool rTouch, uint16_t lx,
		   uint16_t ly, uint16_t rx, uint16_t ry)
{
	touchPackPoint(pts, 0, false, 0, 0);
	touchPackPoint(pts, 1, false, 0, 0);
	if (lTouch && rTouch) {
		touchPackPoint(pts, 0, true, lx, ly);
		touchPackPoint(pts, 1, true, rx, ry);
	} else if (lTouch) {
		touchPackPoint(pts, 0, true, lx, ly);
	} else if (rTouch) {
		touchPackPoint(pts, 0, true, rx, ry);
	}
}
void steamPadsToTouch(uint32_t b, uint16_t touchH, int16_t lpx, int16_t lpy,
		      int16_t rpx, int16_t rpy, uint16_t *lx, uint16_t *ly,
		      uint16_t *rx, uint16_t *ry)
{
	bool lt = (b & TB_LPADT) || (b & TB_LPADC),
	     rt = (b & TB_RPADT) || (b & TB_RPADC);
	*lx = touchHalfX(lpx, false);
	*ly = touchYInv(lpy, touchH);
	*rx = touchHalfX(rpx, true);
	*ry = touchYInv(rpy, touchH);
	if (lt && !(b & TB_LPADT)) {
		*lx = TOUCH_PAD_W / 4;
		*ly = touchH / 2;
	}
	if (rt && !(b & TB_RPADT)) {
		*rx = TOUCH_PAD_W / 4 * 3;
		*ry = touchH / 2;
	}
}

// Map g_back[] paddle code -> Steam button flags (same codes as codeToXB / codeToSwitch).
static void psOrBackCode(uint32_t *b, uint8_t c)
{
	switch (c) {
	case 1:
		*b |= TB_A;
		break;
	case 2:
		*b |= TB_B;
		break;
	case 3:
		*b |= TB_X;
		break;
	case 4:
		*b |= TB_Y;
		break;
	case 5:
		*b |= TB_LB;
		break;
	case 6:
		*b |= TB_RB;
		break;
	case 7:
		*b |= TB_L3;
		break;
	case 8:
		*b |= TB_R3;
		break;
	case 9:
		*b |= TB_VIEW;
		break;
	case 10:
		*b |= TB_QAM;
		break;
	case 11:
		*b |= TB_STEAM;
		break;
	case 12:
		*b |= TB_DUP;
		break;
	case 13:
		*b |= TB_DDN;
		break;
	case 14:
		*b |= TB_DLF;
		break;
	case 15:
		*b |= TB_DRT;
		break;
	case 16:
		*b |= TB_TOUCH;
		break;
	case 17:
		*b |= TB_MUTE;
		break;
	default:
		break;
	}
}
uint32_t psButtonsFromSteam(uint32_t raw)
{
	uint32_t b = raw;
	if (g_qamMap && (b & TB_QAM)) {
		b &= ~(uint32_t)TB_QAM;
		b |= tritonFromCode(g_qamMap);
	}
	if ((b & CHORD_BACK4) == CHORD_BACK4)
		b &= ~(uint32_t)(TB_A | TB_B | TB_X | TB_Y);
	if (b & TB_L4)
		psOrBackCode(&b, g_back[0]);
	if (b & TB_R4)
		psOrBackCode(&b, g_back[1]);
	if (b & TB_L5)
		psOrBackCode(&b, g_back[2]);
	if (b & TB_R5)
		psOrBackCode(&b, g_back[3]);
	return b;
}
// DualSense / DS4 buttons[1]: L1..R3, Create(Share), Options(Start)
uint8_t psShouldersByte(uint32_t b)
{
	return ((b & TB_LB) ? 0x01 : 0) | ((b & TB_RB) ? 0x02 : 0) |
	       ((g_in.lt > SW_TRIG_ON || (b & 0x8000000u)) ? 0x04 : 0) |
	       ((g_in.rt > SW_TRIG_ON || (b & 0x800000u)) ? 0x08 : 0) |
	       ((b & TB_MENU) ? 0x10 : 0) |
	       ((b & TB_VIEW) ? 0x20 : 0) // Menu=Options, View=Create/Share
	       | ((b & TB_L3) ? 0x40 : 0) | ((b & TB_R3) ? 0x80 : 0);
}
uint8_t psHatNibble(uint32_t b)
{
	bool u = b & TB_DUP, d = b & TB_DDN, l = b & TB_DLF, r = b & TB_DRT;
	if (u && r)
		return 1;
	if (r && d)
		return 3;
	if (d && l)
		return 5;
	if (l && u)
		return 7;
	if (u)
		return 0;
	if (r)
		return 2;
	if (d)
		return 4;
	if (l)
		return 6;
	return 8;
}
uint8_t psFaceNibble(uint32_t b)
{
	uint8_t f = 0;
	if (g_abSwap) {
		if (b & TB_A)
			f |= 0x40;
		if (b & TB_B)
			f |= 0x20;
		if (b & TB_X)
			f |= 0x80;
		if (b & TB_Y)
			f |= 0x10;
	} else {
		if (b & TB_A)
			f |= 0x20;
		if (b & TB_B)
			f |= 0x40;
		if (b & TB_X)
			f |= 0x10;
		if (b & TB_Y)
			f |= 0x80;
	}
	return f;
}
