#include "mode_lizard.h"
#include "lizard_map.h"
#include "triton.h"
#include "config.h"
#include <string.h>

// Evaluate the binding table for ONE controller's button word, merging its keyboard/mouse-button/
// consumer outputs into the shared accumulators. Pulled out of rfLizard so every bonded controller can
// feed the same outputs. doRpadMouse/doLpadScroll are map-level (set if such a binding exists at all).
static void lizardEvalSlot(uint32_t buttons, uint8_t &outMod, uint8_t outKeys[6],
			   uint8_t &nKeys, uint8_t &outMBtn,
			   uint8_t &consumerBits, bool &doRpadMouse,
			   bool &doLpadScroll)
{
	// kbdConsumed (per controller): trig bits claimed by a hold-modifier binding so simpler
	// single-button bindings sharing those bits are suppressed for THIS controller.
	uint32_t kbdConsumed = 0;
	for (uint8_t i = 0; i < g_lizardMap.count; i++) {
		const LizardBinding &b = g_lizardMap.bindings[i];
		if (b.outType == LZ_OUT_NONE)
			continue;

		if (b.outType == LZ_OUT_MOUSE_AXIS) {
			if (b.outData[0] == LZ_MSRC_RPAD)
				doRpadMouse = true;
			// LZ_MSRC_LSTICK and LZ_MSRC_GYRO could be added later
			continue;
		}
		if (b.outType == LZ_OUT_SCROLL) {
			if (b.outData[0] == LZ_MSRC_LPAD)
				doLpadScroll = true;
			continue;
		}

		// Check hold modifier (AND) then trigger (any-of OR)
		if (b.holdMask && (buttons & b.holdMask) != b.holdMask)
			continue;
		if (b.trigMask && (buttons & b.trigMask) == 0)
			continue;

		switch (b.outType) {
		case LZ_OUT_MOUSE_BTN:
			if (!b.trigMask || (buttons & b.trigMask))
				outMBtn |= b.outData[0];
			break;

		case LZ_OUT_KBD_CHORD:
			// suppress if trig bits already claimed by a hold-modifier binding
			if (b.holdMask == 0 && b.trigMask != 0 &&
			    (b.trigMask & kbdConsumed))
				break;
			outMod |= b.outData[0];
			for (int j = 1; j < 7 && b.outData[j]; j++) {
				if (nKeys < 6)
					outKeys[nKeys++] = b.outData[j];
			}
			// claim trig bits so simpler bindings don't double-fire
			if (b.holdMask != 0)
				kbdConsumed |= b.trigMask;
			break;

		case LZ_OUT_CONSUMER:
			if (g_usbMode == MODE_LIZARD)
				consumerBits |= b.outData[0];
			break;

		default:
			break;
		}
	}
}

// Add the virtual stick-deflection bits so bindings can trigger on left-stick direction.
static uint32_t lizardButtons(const PuckInput &in)
{
	uint32_t buttons = in.buttons;
	if (in.lx > 12000)
		buttons |= LZ_BTN_LSTICK_RT;
	if (in.lx < -12000)
		buttons |= LZ_BTN_LSTICK_LF;
	if (in.ly < -12000)
		buttons |= LZ_BTN_LSTICK_DN;
	if (in.ly > 12000)
		buttons |= LZ_BTN_LSTICK_UP;
	return buttons;
}

void rfLizard(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev, uint8_t mrid,
	      uint8_t krid)
{
	// ---- merge binding outputs across every bonded controller ----
	uint8_t outMod = 0;
	uint8_t outKeys[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t nKeys = 0;
	uint8_t outMBtn = 0;
	bool doRpadMouse = false, doLpadScroll = false;
	uint8_t consumerBits = 0;

	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used)
			continue;
		lizardEvalSlot(lizardButtons(g_in[s]), outMod, outKeys, nKeys,
			       outMBtn, consumerBits, doRpadMouse, doLpadScroll);
	}

	// ---- right pad -> mouse motion with glide ----
	// Per-slot velocity/last-position so each controller's pad integrates independently; the resulting
	// deltas are SUMMED into one cursor (sub-pixel carry rmx/rmy is shared -- one mouse).
	static int prx[NSLOT] = { 0 }, pry[NSLOT] = { 0 };
	static bool prt[NSLOT] = { false };
	static float vx[NSLOT] = { 0 }, vy[NSLOT] = { 0 }, rmx = 0, rmy = 0;
	int dx = 0, dy = 0;
	if (doRpadMouse) {
		float sumx = 0, sumy = 0;
		float f = g_mFric / 100.0f;
		for (int s = 0; s < NSLOT; s++) {
			if (!g_slot[s].used) {
				vx[s] = vy[s] = 0;
				prt[s] = false;
				continue;
			}
			const PuckInput &in = g_in[s];
			bool rtouch = (in.buttons & TB_RPADT) != 0;
			int rx = in.rpx, ry = in.rpy;
			if (rtouch && prt[s]) {
				vx[s] += (rx - prx[s]);
				vy[s] += (ry - pry[s]);
			}
			if (rtouch) {
				prx[s] = rx;
				pry[s] = ry;
			}
			prt[s] = rtouch;
			sumx += vx[s];
			sumy += vy[s];
			// friction = glide/decay (per slot)
			vx[s] *= f;
			vy[s] *= f;
			if (vx[s] > -1 && vx[s] < 1)
				vx[s] = 0;
			if (vy[s] > -1 && vy[s] < 1)
				vy[s] = 0;
		}
		// Y inverted; *10 = desktop cursor sensitivity (g_mDiv 64 -> eff 640)
		float mxf = sumx / (float)(g_mDiv * 10) + rmx,
		      myf = -(sumy / (float)(g_mDiv * 10)) + rmy;
		dx = (int)mxf;
		dy = (int)myf;
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
	}

	// ---- left pad -> scroll wheel ----
	// Per-slot last-position; accumulator shared (one wheel). Reset only when NO controller is scrolling.
	static int ply[NSLOT] = { 0 };
	static bool plt[NSLOT] = { false };
	static float sacc = 0;
	int dw = 0;
	if (doLpadScroll) {
		bool anyTouch = false;
		for (int s = 0; s < NSLOT; s++) {
			if (!g_slot[s].used) {
				plt[s] = false;
				continue;
			}
			const PuckInput &in = g_in[s];
			bool ltouch = (in.buttons & TB_LPADT) != 0;
			int ly = in.lpy;
			if (ltouch && plt[s])
				sacc += (ly - ply[s]) / (float)(g_mDiv * 24);
			if (ltouch) {
				ply[s] = ly;
				anyTouch = true;
			}
			plt[s] = ltouch;
		}
		if (!anyTouch)
			sacc = 0;
		dw = (int)sacc;
		sacc -= dw;
		if (dw > 15)
			dw = 15;
		if (dw < -15)
			dw = -15;
	}

	// ---- mouse report ----
	static uint8_t pmbtn = 0;
	if (dx || dy || dw || outMBtn != pmbtn) {
		pmbtn = outMBtn;
		hid_mouse_report_t m;
		m.buttons = outMBtn;
		m.x = (int8_t)dx;
		m.y = (int8_t)dy;
		m.wheel = (int8_t)dw;
		m.pan = 0;
		if (mdev->ready())
			mdev->sendReport(mrid, &m, sizeof m);
	}

	// ---- consumer control (edge-triggered) ----
	static uint8_t prevCC = 0;
	if (g_usbMode == MODE_LIZARD) {
		if (consumerBits != prevCC) {
			if (mdev->ready())
				mdev->sendReport(0x03, &consumerBits, 1);
			prevCC = consumerBits;
		}
	}

	// ---- keyboard report ----
	static uint8_t pmod = 0, pkc[6] = { 0, 0, 0, 0, 0, 0 };
	bool chg = (outMod != pmod);
	for (int i = 0; i < 6; i++)
		if (outKeys[i] != pkc[i])
			chg = true;
	if (chg) {
		pmod = outMod;
		for (int i = 0; i < 6; i++)
			pkc[i] = outKeys[i];
		uint8_t krep[8] = { outMod,	0,	    outKeys[0],
				    outKeys[1], outKeys[2], outKeys[3],
				    outKeys[4], outKeys[5] };
		if (kdev->ready())
			kdev->sendReport(krid, krep, 8);
	}
}
