#include "mode_lizard.h"
#include "triton.h"
#include "config.h"

void rfLizard(const uint8_t *r, Adafruit_USBD_HID *mdev,
	      Adafruit_USBD_HID *kdev, uint8_t mrid, uint8_t krid)
{
	uint32_t b = btnsOf(r);
	bool qamHeld = (b & TB_QAM) != 0;
	// --- right pad -> mouse motion with glide (mirrors mode_xinput's rfXboxMouse) ---
	static int prx = 0, pry = 0;
	static bool prt = false;
	static float vx = 0, vy = 0, rmx = 0, rmy = 0;
	bool rtouch = b & TB_RPADT;
	int rx = s16off(r, 22), ry = s16off(r, 24);
	if (rtouch) {
		if (prt) {
			vx += (rx - prx);
			vy += (ry - pry);
		}
		prx = rx;
		pry = ry;
	}
	prt = rtouch;

	// Y inverted; *10 = desktop-cursor sensitivity (g_mDiv 64 -> eff 640). Lower g_mDiv via WebUI slider = faster.
	float mxf = vx / (float)(g_mDiv * 10) + rmx,
	      myf = -(vy / (float)(g_mDiv * 10)) + rmy;
	int dx = (int)mxf, dy = (int)myf;
	rmx = mxf - dx;
	rmy = myf - dy; // sub-pixel carry
	if (dx > 127)
		dx = 127;
	if (dx < -127)
		dx = -127;
	if (dy > 127)
		dy = 127;
	if (dy < -127)
		dy = -127;
	float f = g_mFric / 100.0f;
	vx *= f;
	vy *= f;
	if (vx > -1 && vx < 1)
		vx = 0;
	if (vy > -1 && vy < 1)
		vy = 0; // friction = glide/decay
	// --- left pad -> vertical scroll wheel (no momentum; only while touching, coarse) ---
	static int ply = 0;
	static bool plt = false;
	static float sacc = 0;
	bool ltouch = b & TB_LPADT;
	int ly = s16off(r, 18);
	if (ltouch) {
		if (plt) {
			sacc += (ly - ply) / (float)(g_mDiv * 24);
		}
		ply = ly;
	} else
		sacc = 0;
	plt = ltouch;
	int dw = (int)sacc;
	sacc -= dw;
	if (dw > 15)
		dw = 15;
	if (dw < -15)
		dw = -15; // finger up = wheel up (positive)
	// --- mouse buttons: left=R-pad-click|R-trigger, right=L-trigger, middle=L-pad-click ---
	// trigU8 scales full-range so a full pull reaches ~0xFF; a raw >>8 tops out ~0x80, never crossing threshold.
	uint8_t rtrig = trigU8(u16off(r, 6)), ltrig = trigU8(u16off(r, 4));
	uint8_t mbtn = 0;
	if ((b & TB_RPADC) || rtrig > 180)
		mbtn |= 1; // right trigger -> left (primary) click
	if (ltrig > 180)
		mbtn |= 2; // left trigger  -> right click
	if (b & TB_LPADC)
		mbtn |= 4;
	static uint8_t pmbtn = 0;
	if (dx || dy || dw || mbtn != pmbtn) {
		pmbtn = mbtn;
		hid_mouse_report_t m;
		m.buttons = mbtn;
		m.x = (int8_t)dx;
		m.y = (int8_t)dy;
		m.wheel = (int8_t)dw;
		m.pan = 0;
		if (mdev->ready())
			mdev->sendReport(mrid, &m, sizeof m);
	}
	// --- keyboard: modifiers + up to 6 keycodes ---
	uint8_t mod = 0, kc[6] = { 0, 0, 0, 0, 0, 0 }, nk = 0;
	if (b & TB_LB)
		mod |= KEYBOARD_MODIFIER_LEFTCTRL;
	if (b & TB_RB)
		mod |= KEYBOARD_MODIFIER_LEFTALT;
#define LZK(cond, code)                    \
	do {                               \
		if ((cond) && nk < 6)      \
			kc[nk++] = (code); \
	} while (0)
	LZK(b & TB_A, HID_KEY_ENTER);
	LZK(b & TB_B, HID_KEY_ESCAPE);
	LZK(b & TB_X, HID_KEY_PAGE_UP);
	LZK(b & TB_Y, HID_KEY_PAGE_DOWN);
	LZK(b & TB_VIEW,
	    (g_usbMode == MODE_LIZARD) ? HID_KEY_ESCAPE : HID_KEY_TAB);
	LZK(b & TB_MENU,
	    (g_usbMode == MODE_LIZARD) ? HID_KEY_TAB : HID_KEY_ESCAPE);

	// left stick (XInput sign: +Y = up); deflect ~37% acts as a d-pad
	int sx = s16off(r, 8), sy = s16off(r, 10);
	LZK((b & TB_DUP) || sy > 12000, HID_KEY_ARROW_UP);
	LZK((b & TB_DDN) || sy < -12000, HID_KEY_ARROW_DOWN);
	LZK((b & TB_DLF) || sx < -12000, HID_KEY_ARROW_LEFT);
	LZK((b & TB_DRT) || sx > 12000, HID_KEY_ARROW_RIGHT);
#undef LZK
	if (g_usbMode == MODE_LIZARD) {
		static bool prevL5 = false, prevR5 = false;
		bool mh = (b & TB_STEAM) || qamHeld;
		bool nL5 = mh && (b & TB_L5), nR5 = mh && (b & TB_R5);
		if (nL5 && !prevL5) {
			uint8_t cc = 0x02;
			if (mdev->ready())
				mdev->sendReport(0x03, &cc, 1);
		}
		if (nR5 && !prevR5) {
			uint8_t cc = 0x01;
			if (mdev->ready())
				mdev->sendReport(0x03, &cc, 1);
		}
		if ((!nL5 && prevL5) || (!nR5 && prevR5)) {
			uint8_t cc = 0x00;
			if (mdev->ready())
				mdev->sendReport(0x03, &cc, 1);
		}
		prevL5 = nL5;
		prevR5 = nR5;
		if (mh && (b & TB_X)) {
			mod = KEYBOARD_MODIFIER_LEFTGUI |
			      KEYBOARD_MODIFIER_LEFTCTRL;
			kc[0] = HID_KEY_O;
			kc[1] = 0;
			kc[2] = 0;
			kc[3] = 0;
			kc[4] = 0;
			kc[5] = 0;
			nk = 1;
		}
		if (mh && (b & TB_L4)) {
			mod = KEYBOARD_MODIFIER_LEFTCTRL |
			      KEYBOARD_MODIFIER_LEFTALT;
			kc[0] = HID_KEY_DELETE;
			kc[1] = 0;
			kc[2] = 0;
			kc[3] = 0;
			kc[4] = 0;
			kc[5] = 0;
			nk = 1;
		}
	}
	static uint8_t pmod = 0, pkc[6] = { 0, 0, 0, 0, 0, 0 };
	bool chg = (mod != pmod);
	for (int i = 0; i < 6; i++)
		if (kc[i] != pkc[i])
			chg = true;
	if (chg) {
		pmod = mod;
		for (int i = 0; i < 6; i++)
			pkc[i] = kc[i];
		uint8_t krep[8] = { mod,   0,	  kc[0], kc[1],
				    kc[2], kc[3], kc[4], kc[5] };
		if (kdev->ready())
			kdev->sendReport(krid, krep, 8);
	}
}
