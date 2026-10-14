#include "mode_lizard.h"
#include "lizard_map.h"
#include "triton.h"
#include "config.h"
#include "usb_tx.h"
#include <string.h>

// Evaluate the binding table for ONE controller's button word, merging its keyboard/mouse-button/
// consumer outputs into the shared accumulators. Pulled out of rfLizard so every bonded controller can
// feed the same outputs. mouseSrc/doLpadScroll are map-level (set if such a binding exists at all).
static void lizardEvalSlot(uint64_t buttons, uint8_t &outMod,
			   uint8_t outKeys[6], uint8_t &nKeys, uint8_t &outMBtn,
			   uint8_t &consumerBits, uint8_t &mouseSrc,
			   bool &doLpadScroll)
{
	// kbdConsumed (per controller): trig bits claimed by a hold-modifier binding so simpler
	// single-button bindings sharing those bits are suppressed for THIS controller.
	uint64_t kbdConsumed = 0;
	for (uint8_t i = 0; i < g_lizardMap.count; i++) {
		const LizardBinding &b = g_lizardMap.bindings[i];
		if (b.outType == LZ_OUT_NONE)
			continue;

		if (b.outType == LZ_OUT_MOUSE_AXIS) {
			// bit per LZ_MSRC_* source; gyro is not driven here
			if (b.outData[0] < 8)
				mouseSrc |= (uint8_t)(1u << b.outData[0]);
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
static uint64_t lizardButtons(const PuckInput &in)
{
	uint64_t buttons = (uint64_t)in.buttons;
	if (in.lx > 12000)
		buttons |= LZ_BTN_LSTICK_RT;
	if (in.lx < -12000)
		buttons |= LZ_BTN_LSTICK_LF;
	if (in.ly < -12000)
		buttons |= LZ_BTN_LSTICK_DN;
	if (in.ly > 12000)
		buttons |= LZ_BTN_LSTICK_UP;
	if (in.rx > 12000)
		buttons |= LZ_BTN_RSTICK_RT;
	if (in.rx < -12000)
		buttons |= LZ_BTN_RSTICK_LF;
	if (in.ry < -12000)
		buttons |= LZ_BTN_RSTICK_DN;
	if (in.ry > 12000)
		buttons |= LZ_BTN_RSTICK_UP;
	return buttons;
}

// Per-slot glide/scroll integrators + last-sent report state for rfLizard. These are FILE-SCOPE (not
// function-local statics) so rfLizardRelease() can reset them on a lizard->Steam handoff: without that reset
// a key/mouse-button that was DOWN the instant Steam takes over is never released (the change-dedup below
// thinks it is still "down") and sticks on the desktop until a reconnect/power-cycle.
// Stick deflection -> -1..1 with a deadzone and a squared response.
#define STICK_MOUSE_DEADZONE 3500
#define STICK_MOUSE_PX_PER_S 1100.0f
static float stickAxis(int v)
{
	int a = v < 0 ? -v : v;
	if (a <= STICK_MOUSE_DEADZONE)
		return 0;
	float n = (float)(a - STICK_MOUSE_DEADZONE) /
		  (float)(32767 - STICK_MOUSE_DEADZONE);
	if (n > 1.0f)
		n = 1.0f;
	n *= n;
	return v < 0 ? -n : n;
}

struct PadMouse {
	int px, py; // last touched position
	bool touching;
	float vx, vy; // glide velocity
};
enum { PM_RIGHT, PM_LEFT, PM_N };
static PadMouse g_pm[PM_N][NSLOT];
static float rmx = 0, rmy = 0;
static int ply[NSLOT] = { 0 };
static bool plt[NSLOT] = { false };
static float sacc = 0;
static uint32_t g_stickLastUs = 0;
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
	uint8_t mouseSrc = 0;
	bool doLpadScroll = false;
	uint8_t consumerBits = 0;

	for (int s = 0; s < NSLOT; s++) {
		if (!g_slot[s].used)
			continue;
		lizardEvalSlot(lizardButtons(g_in[s]), outMod, outKeys, nKeys,
			       outMBtn, consumerBits, mouseSrc, doLpadScroll);
	}

	// ---- pads + sticks -> mouse motion ----
	// Every bound source feeds ONE cursor. Pads integrate touch deltas per slot and then glide with friction
	// (state file-scope so rfLizardRelease() can zero it on a handoff); the deltas are SUMMED across
	// controllers and sources, with the sub-pixel carry rmx/rmy shared.
	int dx = 0, dy = 0;
	{
		float sumx = 0, sumy = 0;
		float f = g_mFric / 100.0f;
		for (int p = 0; p < PM_N; p++) {
			const uint8_t src = (p == PM_RIGHT) ?
						    LZ_MSRC_RPAD :
						    LZ_MSRC_LPAD_MOUSE;
			const bool bound = mouseSrc & (1u << src);
			for (int s = 0; s < NSLOT; s++) {
				PadMouse &pm = g_pm[p][s];
				if (!bound || !g_slot[s].used) {
					pm.vx = pm.vy = 0;
					pm.touching = false;
					continue;
				}
				const PuckInput &in = g_in[s];
				bool touch = (in.buttons &
					      (p == PM_RIGHT ? TB_RPADT :
							       TB_LPADT)) != 0;
				int x = (p == PM_RIGHT) ? in.rpx : in.lpx;
				int y = (p == PM_RIGHT) ? in.rpy : in.lpy;
				if (touch && pm.touching) {
					pm.vx += (x - pm.px);
					pm.vy += (y - pm.py);
				}
				if (touch) {
					pm.px = x;
					pm.py = y;
				}
				pm.touching = touch;
				sumx += pm.vx;
				sumy += pm.vy;
				// friction = glide/decay (per slot)
				pm.vx *= f;
				pm.vy *= f;
				if (pm.vx > -1 && pm.vx < 1)
					pm.vx = 0;
				if (pm.vy > -1 && pm.vy < 1)
					pm.vy = 0;
			}
		}
		// Y inverted; *10 = desktop cursor sensitivity (g_mDiv 64 -> eff 640)
		float mxf = sumx / (float)(g_mDiv * 10);
		float myf = -(sumy / (float)(g_mDiv * 10));

		// Sticks are rate controls, not position deltas: scale by elapsed time so the speed is the same
		// whatever the report rate, and square the deflection for fine control near the deadzone.
		if (mouseSrc &
		    ((1u << LZ_MSRC_LSTICK) | (1u << LZ_MSRC_RSTICK))) {
			uint32_t nowUs = micros();
			float dt = g_stickLastUs ?
					   (uint32_t)(nowUs - g_stickLastUs) *
						   1e-6f :
					   0;
			g_stickLastUs = nowUs;
			// a stalled or first report must not fling the cursor
			if (dt > 0.05f)
				dt = 0;
			for (int s = 0; s < NSLOT; s++) {
				if (!g_slot[s].used)
					continue;
				const PuckInput &in = g_in[s];
				float ax = 0, ay = 0;
				if (mouseSrc & (1u << LZ_MSRC_LSTICK)) {
					ax += stickAxis(in.lx);
					ay += stickAxis(in.ly);
				}
				if (mouseSrc & (1u << LZ_MSRC_RSTICK)) {
					ax += stickAxis(in.rx);
					ay += stickAxis(in.ry);
				}
				float speed = STICK_MOUSE_PX_PER_S * 64.0f /
					      (float)g_mDiv;
				mxf += ax * speed * dt;
				myf -= ay * speed * dt;
			}
		} else
			g_stickLastUs = 0;

		mxf += rmx;
		myf += rmy;
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
	// Mouse, consumer and keyboard share ONE endpoint (hid[0]) that stays busy
	// until the host next polls. Sending directly under ready() dropped every
	// report after the first in a cycle while the shadow state still advanced,
	// losing key edges whenever the cursor moved; usbTxHid queues them in order.
	if (dx || dy || dw || outMBtn != pmbtn) {
		pmbtn = outMBtn;
		hid_mouse_report_t m;
		m.buttons = outMBtn;
		m.x = (int8_t)dx;
		m.y = (int8_t)dy;
		m.wheel = (int8_t)dw;
		m.pan = 0;
		usbTxHid(mdev, mrid, &m, sizeof m);
	}

	// ---- consumer control (edge-triggered) ----
	if (g_usbMode == MODE_LIZARD) {
		if (consumerBits != prevCC) {
			usbTxHid(mdev, 0x03, &consumerBits, 1);
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
		usbTxHid(kdev, krid, krep, 8);
	}
}

// Release everything lizard may currently be holding on the host, then clear the last-sent dedup + glide
// state so a later re-entry re-asserts from scratch. Called by puck_hid when Steam TAKES OVER (the
// lizard->gamepad handoff): that transition is a pure runtime flip with NO USB re-enumeration, so a
// keyboard key / mouse button / consumer key that was DOWN at that instant would otherwise stay latched on
// the desktop until a reconnect or power-cycle (the reported "stuck input after switching modes"). Sends the
// neutral reports unconditionally (not change-gated) because the whole point is to override a stale held
// state that the dedup would suppress. Same usbTxHid path rfLizard uses.
void rfLizardRelease(Adafruit_USBD_HID *mdev, Adafruit_USBD_HID *kdev,
		     uint8_t mrid, uint8_t krid)
{
	uint8_t krep[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // all keys + modifiers up
	usbTxHid(kdev, krid, krep, 8);
	hid_mouse_report_t m = { 0, 0, 0, 0, 0 }; // no buttons, no motion/wheel
	usbTxHid(mdev, mrid, &m, sizeof m);
	if (g_usbMode == MODE_LIZARD) {
		uint8_t cc = 0; // release any held consumer-control key
		usbTxHid(mdev, 0x03, &cc, 1);
	}
	// Zero the integrators + last-sent shadows so re-entry starts clean (no cursor fling from stale
	// velocity, and held keys/buttons re-send correctly on the next rfLizard pass).
	memset(g_pm, 0, sizeof g_pm);
	for (int s = 0; s < NSLOT; s++) {
		ply[s] = 0;
		plt[s] = false;
	}
	g_stickLastUs = 0;
	rmx = rmy = sacc = 0;
	pmbtn = 0;
	prevCC = 0;
	pmod = 0;
	for (int i = 0; i < 6; i++)
		pkc[i] = 0;
}
