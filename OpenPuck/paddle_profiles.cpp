// paddle_profiles.cpp -- see paddle_profiles.h for the user-facing behavior.
//
// Runs in two contexts, both the loop task: paddleProfilesOnInput() from the RF decode (rf_link.cpp, inside
// rfLinkTask) and paddleProfilesTask() from loop(). No usbd/ISR access, so no locking is needed. Flash writes
// only ever happen from paddleProfilesTask()/saveCfg(), never from the input path.
#include "paddle_profiles.h"
#include "config.h"
#include "triton.h"
#include "haptics.h"
#include "bonds.h"
#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // USBDevice.suspended()
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

#define PP_FILE "/paddles.bin"
#define PP_MAGIC 0x50 // 'P'
#define PP_VER 1

// Button codes (same code space as TypeCfg.back, see config.h / the panel's TYPE_DEFS labels).
#define CODE_OFF 0
#define CODE_A 1
#define CODE_B 2
#define CODE_X 3
#define CODE_Y 4
#define CODE_LB 5
#define CODE_RB 6
#define CODE_L3 7
#define CODE_R3 8
#define CODE_SELECT 9 // Back / Minus / Create
#define CODE_START 10 // Start / Plus / Options
#define CODE_HOME 11 // Guide / Home / PS
#define CODE_DUP 12
#define CODE_DDN 13
#define CODE_DLF 14
#define CODE_DRT 15
#define CODE_PS_TOUCH 16
#define CODE_CAPTURE 18
#define CODE_LT 19
#define CODE_RT 20

// back[] index order: L4 (upper-left), R4 (upper-right), L5 (lower-left), R5 (lower-right)
static const uint32_t PADDLE_BIT[4] = { TB_L4, TB_R4, TB_L5, TB_R5 };

// Seed layouts for profiles 2..4 the first time this firmware runs (profile 1 = whatever is configured now).
// All reprogrammable from the controller or the panel.
static const uint8_t PRESET[PP_COUNT - 1][4] = {
	{ CODE_Y, CODE_B, CODE_X,
	  CODE_A }, // 2: face buttons (left paddles Y/X, right paddles B/A)
	{ CODE_DUP, CODE_DRT, CODE_DLF, CODE_DDN }, // 3: D-pad
	{ CODE_OFF, CODE_OFF, CODE_OFF, CODE_OFF }, // 4: paddles off
};

struct PpStore {
	uint8_t magic, ver;
	uint8_t active[ET_COUNT];
	uint8_t back[ET_COUNT][PP_COUNT][4];
};
static PpStore s_st;
static bool s_storeDirty = false; // /paddles.bin needs writing
static unsigned long s_saveAt =
	0; // deferred saveCfg deadline (0 = nothing pending)

// ---------------------------------------------------------------- feedback buzz sequencer
#define FB_SHORT_MS 90u
#define FB_LONG_MS 450u
#define FB_GAP_MS 130u
static uint8_t s_fbSlot = 0xFF;
static uint8_t s_fbLeft = 0; // pulses still to play
static uint16_t s_fbOnMs = FB_SHORT_MS;
static bool s_fbIsOn = false;
static unsigned long s_fbNext = 0;

static void feedback(uint8_t slot, uint8_t pulses, uint16_t onMs)
{
	if (slot >= NSLOT)
		return;
	// cut a pulse that's still playing (possibly on another slot) before starting the new pattern
	if (s_fbIsOn && s_fbSlot < NSLOT)
		hapticFeedback(s_fbSlot, false);
	bool wasOn = s_fbIsOn;
	s_fbSlot = slot;
	s_fbLeft = pulses;
	s_fbOnMs = onMs;
	s_fbIsOn = false;
	s_fbNext = millis() + (wasOn ? FB_GAP_MS : 0);
}

static void feedbackTask()
{
	if (s_fbSlot >= NSLOT)
		return;
	unsigned long now = millis();
	if ((long)(now - s_fbNext) < 0)
		return;
	if (s_fbIsOn) {
		hapticFeedback(s_fbSlot, false);
		s_fbIsOn = false;
		if (s_fbLeft)
			s_fbLeft--;
		if (!s_fbLeft) {
			s_fbSlot = 0xFF;
			return;
		}
		s_fbNext = now + FB_GAP_MS;
	} else if (s_fbLeft) {
		hapticFeedback(s_fbSlot, true);
		s_fbIsOn = true;
		s_fbNext = now + s_fbOnMs;
	} else {
		s_fbSlot = 0xFF;
	}
}

// ---------------------------------------------------------------- persistence
static void markChanged()
{
	s_storeDirty = true;
	// Batch rapid cycling / several assignments into one flash write a few seconds after the last change.
	s_saveAt = millis() + 3000u;
	if (!s_saveAt)
		s_saveAt = 1;
}

void paddleProfilesSaveIfDirty()
{
	if (!s_storeDirty)
		return;
	s_storeDirty = false;
	// keep the stored copy of each type's active profile in step with the live config
	for (int et = 0; et < ET_COUNT; et++)
		memcpy(s_st.back[et][s_st.active[et]], g_type[et].back, 4);
	InternalFS.remove(PP_FILE);
	File f(InternalFS);
	if (f.open(PP_FILE, FILE_O_WRITE)) {
		f.write((uint8_t *)&s_st, sizeof s_st);
		f.close();
	}
}

void paddleProfilesLoad()
{
	bool ok = false;
	File f(InternalFS);
	if (f.open(PP_FILE, FILE_O_READ)) {
		ok = f.read((uint8_t *)&s_st, sizeof s_st) ==
			     (int)sizeof s_st &&
		     s_st.magic == PP_MAGIC && s_st.ver == PP_VER;
		f.close();
	}
	if (ok)
		for (int et = 0; et < ET_COUNT; et++)
			if (s_st.active[et] >= PP_COUNT)
				ok = false;
	if (!ok) {
		memset(&s_st, 0, sizeof s_st);
		s_st.magic = PP_MAGIC;
		s_st.ver = PP_VER;
		for (int et = 0; et < ET_COUNT; et++)
			for (int p = 1; p < PP_COUNT; p++)
				memcpy(s_st.back[et][p], PRESET[p - 1], 4);
		// nothing written yet: the first profile switch or assignment creates the file
	}
	// g_type[et].back (cfg.bin) is authoritative for the active profile -- the panel may have edited it.
	for (int et = 0; et < ET_COUNT; et++)
		memcpy(s_st.back[et][s_st.active[et]], g_type[et].back, 4);
}

uint8_t paddleProfileActive(uint8_t et)
{
	return et < ET_COUNT ? s_st.active[et] : 0xFF;
}

static void switchProfile(uint8_t slot, int dir)
{
	uint8_t et = g_etype;
	if (et >= ET_COUNT)
		return;
	uint8_t a = s_st.active[et];
	memcpy(s_st.back[et][a], g_type[et].back,
	       4); // keep any panel edits to the outgoing profile
	a = (uint8_t)((a + PP_COUNT + dir) % PP_COUNT);
	s_st.active[et] = a;
	memcpy(g_type[et].back, s_st.back[et][a], 4);
	applyActiveType();
	markChanged();
	feedback(slot, (uint8_t)(a + 1), FB_SHORT_MS);
}

static void assignPaddle(uint8_t paddle, uint8_t code)
{
	uint8_t et = g_etype;
	if (et >= ET_COUNT || paddle >= 4)
		return;
	g_type[et].back[paddle] = code;
	memcpy(s_st.back[et][s_st.active[et]], g_type[et].back, 4);
	applyActiveType();
	markChanged();
}

// ---------------------------------------------------------------- gestures
// Virtual bits for trigger pulls (bits 30/31 are PS-only virtual targets that the controller never sets, but
// we only use these inside this file's learn accumulator, never in g_in).
#define VB_LT 0x40000000u
#define VB_RT 0x80000000u
#define LEARN_TRIG_ON 200 // of 255: a clear pull, not a brush

// Every physical input that can be a paddle or a paddle's target.
#define LEARN_MASK                                                        \
	(TB_A | TB_B | TB_X | TB_Y | TB_QAM | TB_R3 | TB_VIEW | TB_RB |   \
	 TB_DDN | TB_DRT | TB_DLF | TB_DUP | TB_MENU | TB_L3 | TB_STEAM | \
	 TB_LB | CHORD_BACK4 | VB_LT | VB_RT)

static uint8_t codeForBit(uint32_t bit, uint8_t et)
{
	switch (bit) {
	case TB_A:
		return CODE_A;
	case TB_B:
		return CODE_B;
	case TB_X:
		return CODE_X;
	case TB_Y:
		return CODE_Y;
	case TB_LB:
		return CODE_LB;
	case TB_RB:
		return CODE_RB;
	case TB_L3:
		return CODE_L3;
	case TB_R3:
		return CODE_R3;
	case TB_MENU: // physical Select-side button (see triton.h: the TB_ names are swapped)
		return CODE_SELECT;
	case TB_VIEW: // physical Start-side button
		return CODE_START;
	case TB_STEAM:
		return CODE_HOME;
	case TB_DUP:
		return CODE_DUP;
	case TB_DDN:
		return CODE_DDN;
	case TB_DLF:
		return CODE_DLF;
	case TB_DRT:
		return CODE_DRT;
	case VB_LT:
		return CODE_LT;
	case VB_RT:
		return CODE_RT;
	case TB_QAM:
		// the "..." button stands in for the target type's extra button
		if (et == ET_SWITCH)
			return CODE_CAPTURE;
		if (et == ET_DS4 || et == ET_DS5)
			return CODE_PS_TOUCH;
		return 0xFF; // Xbox has no extra button: not assignable
	default:
		return 0xFF;
	}
}

static inline bool oneBit(uint32_t v)
{
	return v && !(v & (v - 1));
}

// clear TB_* bits from the raw report's buttons field (rep[2..5], little-endian)
static void repClear(uint8_t *rep, uint32_t mask)
{
	for (int i = 0; i < 4; i++)
		rep[2 + i] &= (uint8_t) ~(mask >> (8 * i));
}

#define CMD_NONE 0
#define CMD_NEXT 1
#define CMD_PREV 2
#define CMD_LEARN 3
#define GESTURE_STABLE_MS 40u

static uint8_t s_cmd[NSLOT]; // chord currently held
static unsigned long s_cmdSince[NSLOT];
static bool s_cmdFired[NSLOT]; // fired once for this hold; re-arms on release

static uint8_t s_learnSlot =
	0xFF; // controller in paddle-assign mode (one at a time)
static bool s_learnWaitRelease =
	false; // ignore input until everything is let go
static uint32_t s_learnAcc =
	0; // everything pressed since the last all-released moment
static uint8_t s_learnPaddle = 0xFF; // selected paddle (0..3) awaiting a target
static unsigned long s_learnLastMs = 0;

static void learnEnter(uint8_t slot)
{
	s_learnSlot = slot;
	s_learnWaitRelease = true;
	s_learnAcc = 0;
	s_learnPaddle = 0xFF;
	s_learnLastMs = millis();
	feedback(slot, 1, FB_LONG_MS);
}

static void learnExit()
{
	if (s_learnSlot < NSLOT)
		feedback(s_learnSlot, 1, FB_LONG_MS);
	s_learnSlot = 0xFF;
	s_learnPaddle = 0xFF;
	s_learnAcc = 0;
}

// One tap (everything pressed between two all-released moments) in paddle-assign mode.
static void learnTap(uint8_t slot, uint32_t acc)
{
	uint32_t paddles = acc & CHORD_BACK4, others = acc & ~CHORD_BACK4;
	if (oneBit(paddles) && !others) {
		uint8_t p = 0;
		while (PADDLE_BIT[p] != paddles)
			p++;
		if (p == s_learnPaddle) { // same paddle twice -> paddle off
			assignPaddle(p, CODE_OFF);
			s_learnPaddle = 0xFF;
			feedback(slot, 3, FB_SHORT_MS);
		} else {
			s_learnPaddle = p;
			feedback(slot, 1, FB_SHORT_MS);
		}
		return;
	}
	if (!paddles && oneBit(others) && s_learnPaddle < 4) {
		uint8_t code = codeForBit(others, g_etype);
		if (code != 0xFF) {
			assignPaddle(s_learnPaddle, code);
			s_learnPaddle = 0xFF;
			feedback(slot, 2, FB_SHORT_MS);
		}
	}
	// anything else (several buttons at once, a target with no paddle selected) is ignored
}

void paddleProfilesOnInput(uint8_t slot, uint8_t *rep)
{
	if (slot >= NSLOT)
		return;
	if (g_etype >=
	    ET_COUNT) { // Steam / Lizard / DirectInput / SInput: hands off
		s_learnSlot = 0xFF;
		return;
	}
	PuckInput &in = g_in[slot];
	uint32_t raw = in.buttons;
	unsigned long now = millis();
	bool back4 = (raw & CHORD_BACK4) == CHORD_BACK4;

	// ---- chord detection (also runs in assign mode, so back4 + Start can close it) ----
	uint8_t cmd = CMD_NONE;
	if (back4) {
		if (raw & TB_RB)
			cmd = CMD_NEXT;
		else if (raw & TB_LB)
			cmd = CMD_PREV;
		else if (raw & TB_VIEW)
			cmd = CMD_LEARN;
	}
	if (cmd != s_cmd[slot]) {
		s_cmd[slot] = cmd;
		s_cmdSince[slot] = now;
		s_cmdFired[slot] = false;
	} else if (cmd != CMD_NONE && !s_cmdFired[slot] &&
		   now - s_cmdSince[slot] >= GESTURE_STABLE_MS &&
		   !USBDevice.suspended()) {
		s_cmdFired[slot] = true;
		if (cmd == CMD_LEARN) {
			if (s_learnSlot == slot)
				learnExit();
			else
				learnEnter(slot);
		} else if (s_learnSlot != slot) {
			switchProfile(slot, cmd == CMD_NEXT ? 1 : -1);
		}
	}

	if (s_learnSlot == slot) {
		// ---- paddle-assign mode: watch taps, send the console nothing ----
		uint32_t pressed = raw & LEARN_MASK;
		if (in.lt >= LEARN_TRIG_ON || (raw & TB_L2))
			pressed |= VB_LT;
		if (in.rt >= LEARN_TRIG_ON || (raw & TB_R2))
			pressed |= VB_RT;
		pressed &= LEARN_MASK;
		if (pressed)
			s_learnLastMs = now;
		if (s_learnWaitRelease) {
			if (!pressed)
				s_learnWaitRelease = false;
		} else if (back4) {
			// a back-4 chord is never a tap; let it go first
			s_learnAcc = 0;
			s_learnWaitRelease = true;
		} else if (pressed) {
			s_learnAcc |= pressed;
		} else if (s_learnAcc) {
			learnTap(slot, s_learnAcc);
			s_learnAcc = 0;
		}
		in.buttons = 0;
		in.lt = in.rt = 0;
		repClear(rep, 0xFFFFFFFFu);
		memset(rep + 6, 0,
		       4); // analog triggers (rep[6..9]) for the push-style modes
		return;
	}

	// ---- normal play: hide the gesture buttons while all four paddles are held ----
	if (back4) {
		in.buttons &= ~(uint32_t)(TB_LB | TB_RB | TB_VIEW);
		repClear(rep, TB_LB | TB_RB | TB_VIEW);
	}
}

void paddleProfilesTask()
{
	feedbackTask();
	unsigned long now = millis();
	if (s_learnSlot < NSLOT && now - s_learnLastMs >= PP_LEARN_TIMEOUT_MS)
		learnExit();
	if (s_saveAt && (long)(now - s_saveAt) >= 0) {
		s_saveAt = 0;
		saveCfg(); // also writes /paddles.bin via paddleProfilesSaveIfDirty()
	}
}
