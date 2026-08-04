#include "config.h"
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
}; // back4+B/X/Y -> these modes (A always STEAM); Y defaults to Switch Pro
// back4+D-pad (left/up/right/down). Defaults are the console personalities that ship WITHOUT a config
// interface -- those modes can't be entered from the panel-less side any other way, and back4+A still returns
// to Steam. Configurable like g_chordBtn (WebUSB fields 34..37).
uint8_t g_chordDpad[4] = { MODE_PS3, MODE_DS4_GAME, MODE_PS5_GAME,
			   MODE_SW_HORI };
bool g_persistMode = false;
uint8_t g_bootMode = 0xFF;

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
		bootMode, chordBtn[3], rsvd1;
	// rsvd1: legacy rumble-strength slot (strength now fixed at RUMBLE_SCALE_PCT; ignored -- kept so the
	// on-flash layout is unchanged and an existing cfg.bin still loads).
	// rxWin10: legacy RF tunable slot (window now fixed; ignored). lizKeep: the id9=0 hold enable (see
	// haptics.h LIZKEEP_MS). landAll87: the verbatim-0x87-relay experiment toggle (haptics.h g_landAll87).
	uint8_t rxWin10, lizKeep, landAll87;
	TypeCfg type[ET_COUNT]; // per-emulated-type back/qam/abSwap/padHaptics
	// TAIL (appended after CFG_MAGIC 0xCF shipped): back4+D-pad mode assignments. New tail fields go HERE, at
	// the end, and loadCfg accepts a short file so an upgrade keeps every existing setting -- see CFG_LEN_MIN.
	uint8_t chordDpad[4];
	// per-type trackpad->stick mapping: [et][0] = left pad, [et][1] = right pad (PS_*)
	uint8_t padStick[ET_COUNT][2];
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
		  0, // rsvd1 (ex rumble strength)
		  (uint8_t)(g_rxWin / 10),
		  g_lizKeep,
		  g_landAll87,
		  {},
		  { g_chordDpad[0], g_chordDpad[1], g_chordDpad[2],
		    g_chordDpad[3] },
		  {} };
	for (int i = 0; i < ET_COUNT; i++) {
		c.type[i] = g_type[i];
		c.padStick[i][0] = g_padStickCfg[i][0];
		c.padStick[i][1] = g_padStickCfg[i][1];
	}
	InternalFS.remove(CFG_FILE);
	File f(InternalFS);
	if (f.open(CFG_FILE, FILE_O_WRITE)) {
		f.write((uint8_t *)&c, sizeof c);
		f.close();
	}
}

void loadCfg()
{
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
			// verbatim-0x87-relay experiment toggle (0/1; default off)
			if (c.landAll87 <= 1)
				g_landAll87 = c.landAll87;
			// The poll RX window is now FIXED (g_rxWin is const) -- any persisted rxWin10 is ignored.
		}
		f.close();
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
