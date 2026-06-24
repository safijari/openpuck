#include "config.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

uint8_t g_usbMode = 0;
bool g_xbox = false;
uint8_t g_chordBtn[3] = {
	MODE_LIZARD, MODE_XBOX, MODE_SW_PRO
}; // back4+B/X/Y -> these modes (A always STEAM); Y defaults to Switch Pro
bool g_persistMode = false;
uint8_t g_bootMode = 0xFF;

bool g_debugCdcThisBoot = false;

// persisted one-shot arm, stored in Cfg.rsvd0 (1 = keep CDC for the next boot)
static uint8_t g_debugCdc = 0;

int g_mDiv = 64, g_mFric = 94;

// Per-type button config. back default {5,6,7,8} = L4->LB R4->RB L5->L3 R5->R3 (0..11 buttons, 12..15 D-pad,
// 16/17 PS touch/mute, 18 Switch Capture). Switch differs: QAM defaults to Capture(18), A/B swap on, and
// trackpad haptics off. qamMap 0 = unmapped (hardcoded per-mode behavior).
TypeCfg g_type[ET_COUNT] = {
	/* ET_XBOX   */ { { 5, 6, 7, 8 }, 0, 0, 1 },
	/* ET_SWITCH */ { { 5, 6, 7, 8 }, 18, 1, 0 },
	/* ET_DS4    */ { { 5, 6, 7, 8 }, 0, 0, 1 },
	/* ET_DS5    */ { { 5, 6, 7, 8 }, 0, 0, 1 },
};
uint8_t g_etype = ET_NONE;

// Live mirrors of the active type (puck modes use the harmless defaults below).
uint8_t g_abSwap = 0;
uint8_t g_back[4] = { 5, 6, 7, 8 };
uint8_t g_qamMap = 0;
uint8_t g_padHaptics = 1;

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
		return;
	}
	const TypeCfg &t = g_type[g_etype];
	for (int i = 0; i < 4; i++)
		g_back[i] = t.back[i];
	g_qamMap = t.qamMap;
	g_abSwap = t.abSwap;
	g_padHaptics = t.padHaptics;
}
// rumble strength % (200 = double); adjustable from the WebUSB panel
uint8_t g_rumbleScale = 200;

// poll rate is fixed. Faster than the controller can refresh wastes airtime; slower adds latency. Any rate
// persisted by an older build is ignored and overwritten with the default on boot (see loadCfg).
const uint32_t g_pollUs = POLL_US_DEFAULT;

#define CFG_FILE "/cfg.bin"
// bumped (per-type TypeCfg blob replaces scalar abSwap/back/qamMap): old cfg ignored -> clean defaults on first boot
#define CFG_MAGIC 0xCA
struct Cfg {
	uint8_t magic, mode, mDiv, mFric, rsvd0, pollU100, persistMode,
		bootMode, chordBtn[3], rumbleScale;
	TypeCfg type[ET_COUNT]; // per-emulated-type back/qam/abSwap/padHaptics
}; // rsvd0 = ex-padSmooth, now the one-shot debug-CDC arm

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
		  g_rumbleScale,
		  {} };
	for (int i = 0; i < ET_COUNT; i++)
		c.type[i] = g_type[i];
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
	File f(InternalFS);
	bool consume = false;
	if (f.open(CFG_FILE, FILE_O_READ)) {
		if (f.read((uint8_t *)&c, sizeof c) == (int)sizeof c &&
		    c.magic == CFG_MAGIC) {
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

			// 0 is a valid setting (rumble off)
			g_rumbleScale = c.rumbleScale;
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
