#include "config.h"
#include "storage.h"
#include "triton.h"
#include "rf_link.h" // g_rxWin (poll RX window persisted here)
#include "haptics.h" // g_hapticBlockOn, g_hapticBlockMs
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
#include <stddef.h> // offsetof (CFG_LEN_MIN, the short-file accept threshold)
using namespace Adafruit_LittleFS_Namespace;

uint8_t g_usbMode = 0;
bool g_xbox = false;
uint8_t g_chordBtn[3] = {
	MODE_LIZARD, MODE_XBOX, MODE_SW_PRO
}; // Quick Access+B/X/Y -> these modes (A always STEAM); Y defaults to Switch Pro
// Quick Access+D-pad (left/up/right/down). Defaults are the console personalities that ship WITHOUT a config
// interface -- those modes can't be entered from the panel-less side any other way, and Quick Access+A still returns
// to Steam. Configurable like g_chordBtn (WebUSB fields 34..37).
uint8_t g_chordDpad[4] = { MODE_PS3, MODE_DS4_GAME, MODE_PS5_GAME,
			   MODE_SW_HORI };
bool g_persistMode = false;
uint8_t g_bootMode = 0xFF;

bool g_isMachineInternal = false;

bool g_debugCdcThisBoot = false;

// persisted one-shot arm, stored in Cfg.rsvd0 (1 = keep CDC for the next boot)
static uint8_t g_debugCdc = 0;

int g_mDiv = 64, g_mFric = 94;

// Per-type button config. back default {5,6,7,8} = L4->LB R4->RB L5->L3 R5->R3 (0..11 buttons, 12..15 D-pad,
// 16/17 PS touch/mute, 18 Switch Capture). Switch differs: QAM defaults to Capture(18), A/B swap on, and
// trackpad haptics off. qamMap 0 = unmapped (hardcoded per-mode behavior). ledBright 0 = no override.
// rumble 1 = enabled (default), 0 = host rumble silenced for that type.
TypeCfg g_type[ET_COUNT] = {
	/* ET_XBOX   */ { { 5, 6, 7, 8 }, 0, 0, 1, 0, 1 },
	/* ET_SWITCH */ { { 5, 6, 7, 8 }, 18, 1, 0, 0, 1 },
	/* ET_DS4    */ { { 5, 6, 7, 8 }, 0, 0, 1, 0, 1 },
	/* ET_DS5    */ { { 5, 6, 7, 8 }, 0, 0, 1, 0, 1 },
};
uint8_t g_etype = ET_NONE;

// Trackpad -> stick mapping, off for every type by default (pads keep their touch/mouse behavior).
uint8_t g_padStickCfg[ET_COUNT][2] = {};
uint8_t g_padStick[2] = { PS_OFF, PS_OFF };

// Live mirrors of the active type (puck modes use the harmless defaults below).
uint8_t g_abSwap = 0;
uint8_t g_back[4] = { 5, 6, 7, 8 };
uint8_t g_qamMap = 0;
uint8_t g_padHaptics = 1;
uint8_t g_rumble = 1;
uint8_t g_ledBright = 0;

SwProfiles g_swProfiles = {};
uint8_t g_swDpadHaptics = 1;
uint8_t g_swQamSelect = 18;
uint8_t g_shortcutFlags = 61;
uint8_t g_rumblePresets[3] = { 5, 0, 8 }, g_rumbleSlot = 2;
uint16_t g_strengthSteps[2][3] = { { 200, 300, 500 }, { 200, 300, 500 } };
uint8_t g_strengthSlots[2] = { 0xFF, 0xFF };

uint8_t *swProfileBack(uint8_t profile)
{
	return profile < 4 ? g_swProfiles.back[profile] :
			     g_swProfiles.extraBack[profile - 4];
}

bool rumbleChord(uint8_t slot, uint32_t buttons)
{
	static uint8_t count[NSLOT] = {}, held[NSLOT] = {};
	if (slot >= NSLOT)
		return false;
	const uint32_t keys = TB_A | TB_B | TB_X | TB_Y | TB_DLF | TB_DUP |
			      TB_DRT | TB_DDN;
	uint32_t key = buttons & keys;
	uint8_t choice = key == TB_DLF ? 1 :
			 key == TB_DUP ? 2 :
			 key == TB_DDN ? 3 :
					 0;
	if (!shortcutHeld(buttons) || !(g_shortcutFlags & SHORTCUT_HAPTICS) ||
	    (buttons & TB_MENU) || !choice) {
		count[slot] = held[slot] = 0;
		return false;
	}
	if (held[slot] != choice) {
		held[slot] = choice;
		count[slot] = 0;
	}
	if (count[slot] < 12 && ++count[slot] == 12) {
		uint8_t pulses;
		if (choice == 1) {
			g_rumbleSlot =
				g_rumbleSlot < 3 ? (g_rumbleSlot + 1) % 3 : 0;
			g_rumbleStyle = g_rumblePresets[g_rumbleSlot];
			pulses = g_rumbleSlot + 1;
		} else {
			uint8_t which = choice - 2;
			uint16_t &strength = which == 0 ? g_hdPadScale :
							  g_rumbleScale;
			uint8_t &index = g_strengthSlots[which];
			if (index >= 3 ||
			    g_strengthSteps[which][index] != strength) {
				index = 0xFF;
				for (uint8_t i = 0; i < 3; i++)
					if (g_strengthSteps[which][i] ==
						    strength &&
					    index == 0xFF)
						index = i;
			}
			index = index < 3 ? (index + 1) % 3 : 0;
			strength = g_strengthSteps[which][index];
			pulses = index + 1;
		}
		hapticShortcutFeedback(slot, pulses);
	}
	return true;
}

bool swProfileChord(uint8_t slot, uint32_t buttons)
{
	static uint8_t held[NSLOT] = {}, count[NSLOT] = {};
	const uint32_t keys[7] = { TB_B,   TB_X,   TB_Y,  TB_DLF,
				   TB_DUP, TB_DRT, TB_DDN };
	if (slot >= NSLOT)
		return false;
	uint8_t target = 0;
	if ((g_shortcutFlags & SHORTCUT_PROFILES) && g_swProfiles.enabled &&
	    shortcutHeld(buttons) && !(buttons & (TB_A | TB_MENU))) {
		for (uint8_t i = 0; i < 7; i++) {
			if ((!(g_shortcutFlags & SHORTCUT_HAPTICS) ||
			     (i != 3 && i != 4 && i != 6)) &&
			    (buttons & keys[i])) {
				target = g_swProfiles.chord[i];
				break;
			}
		}
	}
	if (!target) {
		held[slot] = count[slot] = 0;
		return false;
	}
	if (target != held[slot]) {
		held[slot] = target;
		count[slot] = 0;
	}
	if (count[slot] < 12 && ++count[slot] == 12) {
		g_swProfiles.active = target - 1;
		applyActiveType();
		shortcutModeRequest(MODE_SW_PRO, slot);
	}
	return true;
}

void captureFeedbackChord(uint8_t slot, uint32_t buttons)
{
	static bool held[NSLOT] = {};
	if (slot >= NSLOT)
		return;
	bool active = g_usbMode == MODE_SW_PRO && g_swQamSelect &&
		      (g_shortcutFlags & SHORTCUT_CAPTURE) &&
		      (buttons & (TB_QAM | TB_MENU)) == (TB_QAM | TB_MENU);
	if (active && !held[slot])
		hapticShortcutFeedback(slot, 1);
	held[slot] = active;
}

void applyActiveType()
{
	g_etype = etypeForMode(g_usbMode);
	if (g_etype >=
	    ET_COUNT) { // puck mode (Steam/Lizard): no remap, haptics on
		g_back[0] = 5;
		g_back[1] = 6;
		g_back[2] = 7;
		g_back[3] = 8;
		g_qamMap = 0;
		g_abSwap = 0;
		g_padHaptics = 1;
		g_rumble = 1;
		g_ledBright = 0;
		g_padStick[0] = g_padStick[1] = PS_OFF;
		return;
	}
	const TypeCfg &t = g_type[g_etype];
	for (int i = 0; i < 4; i++)
		g_back[i] = t.back[i];
	if (g_usbMode == MODE_SW_PRO && g_swProfiles.enabled)
		for (int i = 0; i < 4; i++)
			g_back[i] = swProfileBack(g_swProfiles.active)[i];
	g_qamMap = t.qamMap;
	g_abSwap = t.abSwap;
	g_padHaptics = t.padHaptics;
	g_rumble = t.rumble;
	g_ledBright = t.ledBright;
	g_padStick[0] = g_padStickCfg[g_etype][0];
	g_padStick[1] = g_padStickCfg[g_etype][1];
}
// poll rate defaults to POLL_US_DEFAULT (250 Hz), matching the real Valve puck (see config.h). The
// delivered report rate equals the poll rate (fresh IMU in every reply). Live-adjustable via console
// "PR<hz>" for on-HW sweeps; session-only, so any rate persisted by an older build is ignored and boot
// always starts at the default (see loadCfg).
uint32_t g_pollUs = POLL_US_DEFAULT;

#define CFG_FILE "/cfg.bin"
// Struct layout/semantics changed (TypeCfg gained rumble byte); bump so old flash format is discarded ->
// clean defaults once.
#define CFG_MAGIC 0xCF
struct Cfg {
	uint8_t magic, mode, mDiv, mFric, rsvd0, pollU100, persistMode,
		bootMode, chordBtn[3], rumbScale2;
	// rumbScale2: host-rumble strength as PERCENT/2 (so 500% fits a byte). 0 = never set -> keep the
	// RUMBLE_SCALE_PCT default. This revives the byte the removed rumble-strength slider used, so the
	// on-flash layout is unchanged and an existing cfg.bin still loads.
	// rxWin10: legacy RF tunable slot (window now fixed; ignored). lizKeep: the id9=0 hold enable (see
	// haptics.h LIZKEEP_MS).
	uint8_t rxWin10, lizKeep, isMachineInternal;
	TypeCfg type[ET_COUNT]; // per-emulated-type back/qam/abSwap/padHaptics
	// TAIL (appended after CFG_MAGIC 0xCF shipped): Quick Access+D-pad mode assignments. New tail fields go HERE, at
	// the end, and loadCfg accepts a short file so an upgrade keeps every existing setting -- see CFG_LEN_MIN.
	uint8_t chordDpad[4];
	// per-type trackpad->stick mapping: [et][0] = left pad, [et][1] = right pad (PS_*)
	uint8_t padStick[ET_COUNT][2];
	// RUMBLE_STYLE_*; 0xFF (short pre-tail file) -> compiled default
	uint8_t rumbleStyle;
	// autonomous controller power-off on host sleep (see haptics.h g_suspendOff). 0/1; 0xFF (short
	// pre-tail file) -> compiled default (on)
	uint8_t suspendOff;
	SwProfiles swProfiles;
	uint8_t swDpadHaptics;
	uint8_t hdPadScale2, reservedRumbleStyles[2], reservedWaveform;
	uint8_t reservedWavePresets[2];
	uint8_t reservedWaveThird, reservedWaveSlot;
	uint8_t swQamSelect;
	uint8_t shortcutFlags, rumblePresets[3], strengthSteps[2][3];
	uint8_t strengthSlots[2], rumbleSlot;
}; // rsvd0 = ex-padSmooth, now the one-shot debug-CDC arm

// Shortest cfg.bin we still accept: the layout as of CFG_MAGIC 0xCF, i.e. everything before the appended tail.
// A file that stops anywhere in the tail leaves those bytes at the 0xFF prefill loadCfg() applies, which every
// tail field treats as "unset" and replaces with its default.
#define CFG_LEN_MIN (offsetof(struct Cfg, chordDpad))

void saveCfg()
{
	Cfg c = { CFG_MAGIC,
		  g_usbMode,
		  (uint8_t)g_mDiv,
		  (uint8_t)g_mFric,
		  g_debugCdc,
		  (uint8_t)(g_pollUs / 100),
		  (uint8_t)(g_persistMode ? 1 : 0),
		  g_bootMode,
		  { g_chordBtn[0], g_chordBtn[1], g_chordBtn[2] },
		  (uint8_t)(g_rumbleScale / 2), // host-rumble strength, pct/2
		  (uint8_t)(g_rxWin / 10),
		  g_lizKeep,
		  (uint8_t)(g_isMachineInternal ? 0xEE : 0),
		  {},
		  { g_chordDpad[0], g_chordDpad[1], g_chordDpad[2],
		    g_chordDpad[3] },
		  {},
		  g_rumbleStyle,
		  g_suspendOff,
		  g_swProfiles,
		  g_swDpadHaptics,
		  (uint8_t)(g_hdPadScale / 2),
		  { 0xFF, 0xFF },
		  0xFF,
		  { 0xFF, 0xFF },
		  0xFF,
		  0xFF,
		  g_swQamSelect,
		  g_shortcutFlags,
		  { g_rumblePresets[0], g_rumblePresets[1],
		    g_rumblePresets[2] },
		  {},
		  { g_strengthSlots[0], g_strengthSlots[1] },
		  g_rumbleSlot };
	for (int i = 0; i < ET_COUNT; i++) {
		c.type[i] = g_type[i];
		c.padStick[i][0] = g_padStickCfg[i][0];
		c.padStick[i][1] = g_padStickCfg[i][1];
	}
	for (uint8_t w = 0; w < 2; w++)
		for (uint8_t i = 0; i < 3; i++)
			c.strengthSteps[w][i] = g_strengthSteps[w][i] / 2;
	storageWriteFile(CFG_FILE, "/cfg.tmp", (const uint8_t *)&c, sizeof c);
}

static void swProfilesLoad(const SwProfiles &saved, const uint8_t defaults[4])
{
	g_swProfiles = {};
	for (int p = 0; p < SW_PROFILE_COUNT; p++)
		memcpy(swProfileBack(p), defaults, 4);
	if (saved.enabled > 1 || saved.active >= SW_PROFILE_COUNT)
		return;
	for (int p = 0; p < 4; p++)
		for (int k = 0; k < 4; k++)
			if (saved.back[p][k] > 20)
				return;
	for (int i = 0; i < 7; i++)
		if (saved.chord[i] > SW_PROFILE_COUNT)
			return;
	// The prefix stays compatible with configs that lack extraBack.
	memcpy(&g_swProfiles, &saved, 25);
	bool extraValid = true;
	for (int p = 0; p < 3; p++)
		for (int k = 0; k < 4; k++)
			if (saved.extraBack[p][k] > 20)
				extraValid = false;
	if (extraValid)
		memcpy(g_swProfiles.extraBack, saved.extraBack, 12);
}

void loadCfg()
{
	if (g_storageState == 0) {
		applyActiveType();
		return;
	}
	Cfg c;
	// 0xFF prefill: bytes a SHORT (pre-tail) cfg.bin never wrote stay 0xFF, which is not a valid mode/flag, so
	// each tail field below falls back to its compiled default instead of reading whatever was on the stack.
	memset(&c, 0xFF, sizeof c);
	File f(InternalFS);
	bool consume = false;
	if (f.open(CFG_FILE, FILE_O_READ)) {
		int got = f.read((uint8_t *)&c, sizeof c);
		// Accept a file that is short only in the appended tail (>= CFG_LEN_MIN): an upgrade from a build
		// predating the tail keeps mode/paddles/chords instead of silently reverting to factory defaults.
		// Anything shorter, or a stale magic, is a real layout change -> discard and use defaults.
		if (got >= (int)CFG_LEN_MIN && c.magic == CFG_MAGIC) {
			g_mDiv = c.mDiv ? c.mDiv : 64;
			g_mFric = c.mFric;
			for (int i = 0; i < ET_COUNT; i++)
				g_type[i] = c.type[i];
			g_persistMode = c.persistMode ? true : false;
			// one-shot debug-CDC (Cfg.rsvd0): honor for THIS boot, then consume so the next boot reverts to normal.
			g_debugCdcThisBoot = c.rsvd0 ? true : false;
			if (c.rsvd0) {
				g_debugCdc = 0;
				consume = true;
			}
			// poll rate is fixed; rewrite cfg so the persisted byte matches the new default.
			if (c.pollU100 != (uint8_t)(POLL_US_DEFAULT / 100))
				consume = true;
			// boot-mode policy: a one-shot bootMode (explicit switch when !persist) wins once then clears;
			// otherwise persist->last mode, else->Steam.
			if (c.bootMode != 0xFF) {
				g_usbMode = modeValid(c.bootMode) ? c.bootMode :
								    0;
				consume = true;
			} else
				g_usbMode = g_persistMode ? (modeValid(c.mode) ?
								     c.mode :
								     0) :
							    0;
			static const uint8_t CHORD_DEF[3] = { MODE_LIZARD,
							      MODE_XBOX,
							      MODE_SW_PRO };
			for (int i = 0; i < 3; i++)
				g_chordBtn[i] = modeValid(c.chordBtn[i]) ?
							c.chordBtn[i] :
							CHORD_DEF[i];
			// D-pad chords: 0xFF (short pre-tail file) or any invalid mode keeps the compiled default.
			for (int i = 0; i < 4; i++)
				if (modeValid(c.chordDpad[i]))
					g_chordDpad[i] = c.chordDpad[i];
			// Pad->stick mapping: 0xFF (a file predating this tail field) or an out-of-range
			// value keeps the compiled default (off).
			for (int i = 0; i < ET_COUNT; i++)
				for (int k = 0; k < 2; k++)
					if (c.padStick[i][k] <= PS_MAX)
						g_padStickCfg[i][k] =
							c.padStick[i][k];
			// grow a short file to the current layout on the next save
			if (got < (int)sizeof c)
				consume = true;

			// lizard-suppression keepalive enable (0/1; anything else = a pre-0xCE cfg leaked
			// through -> keep the on default)
			if (c.lizKeep <= 1)
				g_lizKeep = c.lizKeep;

			// This variable previously used to have values 0/1.
			// Use a seperate trigger value 0xEE if we want to emulate a Steam Machine's
			// internal receiver.
			g_isMachineInternal = (c.isMachineInternal == 0xEE);

			// host-rumble strength (pct/2; 0 = never set, or a cfg.bin from before this
			// field was revived -> keep the RUMBLE_SCALE_PCT default)
			if (c.rumbScale2) {
				uint16_t pct = (uint16_t)c.rumbScale2 * 2;
				if (pct < RUMBLE_SCALE_MIN)
					pct = RUMBLE_SCALE_MIN;
				else if (pct > RUMBLE_SCALE_MAX)
					pct = RUMBLE_SCALE_MAX;
				g_rumbleScale = pct;
			}
			// host-rumble style (0xFF = short pre-tail file -> keep the default)
			if (c.rumbleStyle <= RUMBLE_STYLE_MAX)
				g_rumbleStyle = c.rumbleStyle;
			// suspend power-off enable (0xFF = a cfg.bin predating this tail field -> keep the on default)
			if (c.suspendOff <= 1)
				g_suspendOff = c.suspendOff;
			// The poll RX window is now FIXED (g_rxWin is const) -- any persisted rxWin10 is ignored.
		}
		f.close();
	}
	swProfilesLoad(c.swProfiles, g_type[ET_SWITCH].back);
	if (c.swDpadHaptics <= 1)
		g_swDpadHaptics = c.swDpadHaptics;
	if (c.hdPadScale2 <= 250)
		g_hdPadScale = (uint16_t)c.hdPadScale2 * 2;
	if (g_rumbleStyle == 7)
		g_rumbleStyle = 8;
	if (c.swQamSelect <= 20)
		g_swQamSelect = c.swQamSelect;
	if (c.shortcutFlags <= 63)
		g_shortcutFlags = c.shortcutFlags;
	else
		g_shortcutFlags = 61 | (g_swProfiles.enabled ? 2 : 0);
	for (uint8_t i = 0; i < 3; i++) {
		if (c.rumblePresets[i] <= 6 || c.rumblePresets[i] == 8)
			g_rumblePresets[i] = c.rumblePresets[i];
		for (uint8_t w = 0; w < 2; w++)
			if (c.strengthSteps[w][i] <= 250 &&
			    (w == 0 || c.strengthSteps[w][i] >= 5))
				g_strengthSteps[w][i] =
					c.strengthSteps[w][i] * 2;
	}
	g_rumbleSlot = 0xFF;
	for (uint8_t i = 0; i < 3; i++)
		if (g_rumblePresets[i] == g_rumbleStyle && g_rumbleSlot == 0xFF)
			g_rumbleSlot = i;
	if (c.rumbleSlot < 3 && g_rumblePresets[c.rumbleSlot] == g_rumbleStyle)
		g_rumbleSlot = c.rumbleSlot;
	for (uint8_t w = 0; w < 2; w++) {
		uint16_t strength = w == 0 ? g_hdPadScale : g_rumbleScale;
		g_strengthSlots[w] = 0xFF;
		for (uint8_t i = 0; i < 3; i++)
			if (g_strengthSteps[w][i] == strength &&
			    g_strengthSlots[w] == 0xFF)
				g_strengthSlots[w] = i;
		if (c.strengthSlots[w] < 3 &&
		    g_strengthSteps[w][c.strengthSlots[w]] == strength)
			g_strengthSlots[w] = c.strengthSlots[w];
	}
	// resolve the active emulated type's settings into the live mirrors the mode builders read
	applyActiveType();
	// clear the one-shot so the NEXT cold boot reverts to the default/persist policy
	if (consume) {
		g_bootMode = 0xFF;
		saveCfg();
	}
}

void saveMode(uint8_t m)
{
	if (g_persistMode) {
		g_usbMode = m;
		g_bootMode = 0xFF;
	} else {
		g_bootMode = m;
	}
	saveCfg();
}

void armDebugCdcNextBoot()
{
	g_debugCdc = 1;
	saveCfg();
} // next boot keeps CDC; loadCfg() consumes it after

// FULL factory wipe: reformat the internal LittleFS, erasing cfg.bin (modes/tunables/chords) AND bonds.bin
// (paired-controller record). Caller reboots: next boot finds no files and falls back to clean defaults, and
// the controller must be re-paired. Irreversible -- gated behind explicit confirmation at every call site.
void factoryErase()
{
	// ensure mounted before we reformat (no-op if already up)
	InternalFS.begin();
	InternalFS.format();
}

// One-time factory reset for the -DOPK_FACTORY_RESET recovery build: clear a bad config/bond ONCE (first boot
// after flashing) then persist normally. "Already reset" is tracked by a tag file holding the build's git hash,
// written AFTER the wipe (so it survives in the freshly-formatted FS):
//   - tag missing or != this build's hash  -> wipe, then stamp the tag. Next boot persists.
//   - tag == this build's hash             -> already reset for this build: skip, boot normally.
// Keying the tag to the git hash means flashing a DIFFERENT build re-triggers the wipe. buildTag is OPK_GIT_HASH.
#define RESET_TAG_FILE "/rsttag"
void factoryResetOnce(const char *buildTag)
{
	char tag[24] = { 0 };
	{
		File f(InternalFS);
		if (f.open(RESET_TAG_FILE, FILE_O_READ)) {
			int n = f.read((uint8_t *)tag, sizeof tag - 1);
			if (n > 0)
				tag[n] = 0;
			f.close();
		}
	}
	if (strncmp(tag, buildTag, sizeof tag - 1) == 0)
		return; // this build already did its one-time reset -> persist
	factoryErase(); // wipe cfg.bin + bonds.bin + the old tag
	InternalFS.begin(); // remount the fresh FS
	File g(InternalFS); // stamp the tag so subsequent boots skip the wipe
	if (g.open(RESET_TAG_FILE, FILE_O_WRITE)) {
		g.write((const uint8_t *)buildTag, strlen(buildTag));
		g.close();
	}
}
