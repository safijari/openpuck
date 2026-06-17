#include "config.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>
using namespace Adafruit_LittleFS_Namespace;

uint8_t g_usbMode = 0;
bool    g_xbox = false;
uint8_t g_chordBtn[3] = { MODE_LIZARD, MODE_XBOX, MODE_SW_PRO };   // back4+B/X/Y -> these modes (A always STEAM); Y defaults to Switch Pro
bool    g_persistMode = false;
uint8_t g_bootMode = 0xFF;

bool          g_debugCdcThisBoot = false;
static uint8_t g_debugCdc = 0;   // persisted one-shot arm, stored in Cfg.rsvd0 (1 = keep CDC for the next boot)

int     g_mDiv = 64, g_mFric = 94;
uint8_t g_abSwap = 0;
uint8_t g_back[4] = {5,6,7,8};   // L4->LB R4->RB L5->L3 R5->R3 (0..11 buttons, 12..15 D-pad U/D/L/R)
uint8_t g_qamMap = 0;            // 0 = default (unmapped, uses hardcoded behavior per mode)
uint8_t g_rumbleScale = 200;     // rumble strength % (200 = double); adjustable from the WebUSB panel

// poll rate is fixed. Faster than the controller can refresh wastes airtime; slower adds latency. Any rate
// persisted by an older build is ignored and overwritten with the default on boot (see loadCfg).
const uint32_t g_pollUs = POLL_US_DEFAULT;

#define CFG_FILE "/cfg.bin"
#define CFG_MAGIC 0xC9   // bumped (rumbleScale): old cfg ignored -> clean defaults on first boot
struct Cfg { uint8_t magic, mode, mDiv, mFric, rsvd0, abSwap, back[4], pollU100, persistMode, bootMode, chordBtn[3], qamMap, rumbleScale; };  // rsvd0 = ex-padSmooth, now the one-shot debug-CDC arm

void saveCfg(){
  Cfg c={CFG_MAGIC,g_usbMode,(uint8_t)g_mDiv,(uint8_t)g_mFric,g_debugCdc,g_abSwap,
         {g_back[0],g_back[1],g_back[2],g_back[3]},(uint8_t)(g_pollUs/100),(uint8_t)(g_persistMode?1:0),g_bootMode,
         {g_chordBtn[0],g_chordBtn[1],g_chordBtn[2]},g_qamMap,g_rumbleScale};
  InternalFS.remove(CFG_FILE); File f(InternalFS);
  if(f.open(CFG_FILE,FILE_O_WRITE)){ f.write((uint8_t*)&c,sizeof c); f.close(); }
}

void loadCfg(){
  Cfg c; File f(InternalFS); bool consume=false;
  if(f.open(CFG_FILE,FILE_O_READ)){
    if(f.read((uint8_t*)&c,sizeof c)==(int)sizeof c && c.magic==CFG_MAGIC){
      g_mDiv=c.mDiv?c.mDiv:64; g_mFric=c.mFric;
      g_abSwap=c.abSwap; for(int i=0;i<4;i++) g_back[i]=c.back[i];
      g_persistMode = c.persistMode?true:false;
      // one-shot debug-CDC (Cfg.rsvd0): honor it for THIS boot, then consume so the next boot reverts to normal.
      g_debugCdcThisBoot = c.rsvd0 ? true : false;
      if(c.rsvd0){ g_debugCdc = 0; consume = true; }
      // poll rate is fixed (g_pollUs const = POLL_US_DEFAULT). A stale rate from an older build is never
      // applied; rewrite cfg so the persisted byte also matches the new default.
      if(c.pollU100 != (uint8_t)(POLL_US_DEFAULT/100)) consume=true;
      // boot-mode policy: a one-shot bootMode (set by an explicit switch when !persist) wins once and is then
      // cleared; otherwise persist->last mode, else->Steam. (poll rate stays POLL_US_DEFAULT -- never restored from cfg.)
      if(c.bootMode!=0xFF){ g_usbMode=modeValid(c.bootMode)?c.bootMode:0; consume=true; }
      else                 g_usbMode = g_persistMode ? (modeValid(c.mode)?c.mode:0) : 0;
      static const uint8_t CHORD_DEF[3]={MODE_LIZARD,MODE_XBOX,MODE_SW_PRO};
      for(int i=0;i<3;i++) g_chordBtn[i]=modeValid(c.chordBtn[i])?c.chordBtn[i]:CHORD_DEF[i];
      g_qamMap = c.qamMap;
      g_rumbleScale = c.rumbleScale;   // 0 is a valid setting (rumble off)
    }
    f.close();
  }
  if(consume){ g_bootMode=0xFF; saveCfg(); }   // clear the one-shot so the NEXT cold boot reverts to the default/persist policy
}

void saveMode(uint8_t m){
  if(g_persistMode){ g_usbMode=m; g_bootMode=0xFF; }
  else             { g_bootMode=m; }
  saveCfg();
}

void armDebugCdcNextBoot(){ g_debugCdc = 1; saveCfg(); }   // next boot keeps CDC; loadCfg() consumes it after

// FULL factory wipe: erase ALL persistent storage in one shot -- cfg.bin (modes/tunables/chords), bonds.bin
// (the paired-controller record), and anything else living in the internal LittleFS. format() reformats the
// whole filesystem, so there's nothing left to selectively miss. The caller is expected to reboot immediately
// (NVIC_SystemReset): on the next boot loadCfg()/loadBonds() find no files and fall back to clean defaults,
// and the controller must be re-paired. Destructive and irreversible -- gated behind explicit confirmation at
// every call site (serial "ERASE-ALL", WebUSB op 0x0A magic, both with on-screen warnings).
void factoryErase(){
  InternalFS.begin();      // ensure mounted before we reformat (no-op if already up)
  InternalFS.format();     // wipes cfg.bin + bonds.bin + the entire FS
}

// One-time factory reset for the -DOPK_FACTORY_RESET recovery build. The point is to clear a bad config/bond
// ONCE (at the first boot after flashing) and then behave like a normal build that PERSISTS settings -- NOT to
// wipe on every boot (which would make the board un-configurable). We track "already reset" with a tiny tag file
// holding the build's git hash, written AFTER the wipe (so it survives in the freshly-formatted FS):
//   - tag missing or != this build's hash  -> a reset is pending: wipe, then stamp the tag. Next boot persists.
//   - tag == this build's hash             -> already reset for this build: skip, boot normally.
// Because the tag is keyed to the git hash, flashing a DIFFERENT build re-triggers the one-time wipe; re-flashing
// the identical image does not (use the WebUSB/serial erase for an on-demand wipe). buildTag is OPK_GIT_HASH.
#define RESET_TAG_FILE "/rsttag"
void factoryResetOnce(const char* buildTag){
  char tag[24] = {0};
  { File f(InternalFS);
    if (f.open(RESET_TAG_FILE, FILE_O_READ)) { int n=f.read((uint8_t*)tag, sizeof tag - 1); if(n>0) tag[n]=0; f.close(); } }
  if (strncmp(tag, buildTag, sizeof tag - 1) == 0) return;   // this build already did its one-time reset -> persist
  factoryErase();                                            // wipe cfg.bin + bonds.bin + the old tag
  InternalFS.begin();                                        // remount the fresh FS
  File g(InternalFS);                                        // stamp the tag so subsequent boots skip the wipe
  if (g.open(RESET_TAG_FILE, FILE_O_WRITE)) { g.write((const uint8_t*)buildTag, strlen(buildTag)); g.close(); }
}
