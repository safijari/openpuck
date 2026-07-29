#include "mode_lizard.h"
#include "lizard_map.h"
#include "triton.h"
#include "config.h"
#include <string.h>

// Evaluate the binding table for ONE controller's button word, merging its keyboard/mouse-button/
// consumer outputs into the shared accumulators. Pulled out of rfLizard so every bonded controller can
// feed the same outputs. doRpadMouse/doLpadScroll are map-level (set if such a binding exists at all).
static void lizardEvalSlot(uint32_t buttons, uint8_t &outMod,
			   uint8_t outKeys[6], uint8_t &nKeys, uint8_t &outMBtn,
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
// The LZ_BTN_LSTICK_* flags live on bits 28..31, which OVERLAP real controller inputs: bit 28 = right
// grip touch, bit 29 = left grip touch (PROTOCOL.md §8). Merely holding the controller sets the grip
// bits, which would masquerade as an L-stick deflection and fire the bound key. Clear the top 4 bits of
// the physical word (grips are not a bindable lizard input) so these flags reflect ONLY stick deflection.
static uint32_t lizardButtons(const PuckInput &in)
{
	uint32_t buttons = in.buttons & 0x0FFFFFFFu;
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

// Per-slot glide/scroll integrators + last-sent report state for rfLizard. These are FILE-SCOPE (not
// function-local statics) so rfLizardRelease() can reset them on a lizard->Steam handoff: without that reset
// a key/mouse-button that was DOWN the instant Steam takes over is never released (the change-dedup below
// thinks it is still "down") and sticks on the desktop until a reconnect/power-cycle.
static int prx[NSLOT] = { 0 }, pry[NSLOT] = { 0 };
static bool prt[NSLOT] = { false };
static float vx[NSLOT] = { 0 }, vy[NSLOT] = { 0 }, rmx = 0, rmy = 0;
static int ply[NSLOT] = { 0 };
static bool plt[NSLOT] = { false };
static float sacc = 0;
static uint8_t pmbtn = 0; // last-sent mouse buttons
static uint8_t prevCC = 0; // last-sent consumer bits
static uint8_t pmod = 0, pkc[6] = {
	0, 0, 0, 0, 0, 0
}; // last-sent kbd modifier + keycodes

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
			       outMBtn, consumerBits, doRpadMouse,
			       doLpadScroll);
	}

	// ---- right pad -> mouse motion with glide ----
	// Per-slot velocity/last-position so each controller's pad integrates independently; the resulting
	// deltas are SUMMED into one cursor (sub-pixel carry rmx/rmy is shared -- one mouse). State is
	// file-scope (see the block above rfLizard) so rfLizardRelease() can zero it on a handoff.
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
	// State is file-scope (see the block above rfLizard) so rfLizardRelease() can zero it on a handoff.
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
	if (g_usbMode == MODE_LIZARD) {
		if (consumerBits != prevCC) {
			if (mdev->ready())
				mdev->sendReport(0x03, &consumerBits, 1);
			prevCC = consumerBits;
		}
	}

	// ---- keyboard report ----
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

// Release everything lizard may currently be holding on the host, then clear the last-sent dedup + glide
// state so a later re-entry re-asserts from scratch. Called by puck_hid when Steam TAKES OVER (the
// lizard->gamepad handoff): that transition is a pure runtime flip with NO USB re-enumeration, so a
// keyboard key / mouse button / consumer key that was DOWN at that instant would otherwise stay latched on
// the desktop until a reconnect or power-cycle (the reported "stuck input after switching modes"). Sends the
// neutral reports unconditionally (not change-gated) because the whole point is to override a stale held
// state that the dedup would suppress. Same direct-sendReport path rfLizard uses.
void rfLizardRelease(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev,
		     uint8_t mrid, uint8_t krid)
{
	uint8_t krep[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // all keys + modifiers up
	if (kdev->ready())
		kdev->sendReport(krid, krep, 8);
	hid_mouse_report_t m = { 0, 0, 0, 0, 0 }; // no buttons, no motion/wheel
	if (mdev->ready())
		mdev->sendReport(mrid, &m, sizeof m);
	if (g_usbMode == MODE_LIZARD) {
		uint8_t cc = 0; // release any held consumer-control key
		if (mdev->ready())
			mdev->sendReport(0x03, &cc, 1);
	}
	// Zero the integrators + last-sent shadows so re-entry starts clean (no cursor fling from stale
	// velocity, and held keys/buttons re-send correctly on the next rfLizard pass).
	for (int s = 0; s < NSLOT; s++) {
		prx[s] = pry[s] = 0;
		prt[s] = false;
		vx[s] = vy[s] = 0;
		ply[s] = 0;
		plt[s] = false;
	}
	rmx = rmy = sacc = 0;
	pmbtn = 0;
	prevCC = 0;
	pmod = 0;
	for (int i = 0; i < 6; i++)
		pkc[i] = 0;
}
