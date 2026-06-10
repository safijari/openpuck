#include "config.h"
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

uint8_t g_usbMode = 0;
bool    g_xbox = false;
uint8_t g_chordBtn[3] = { MODE_LIZARD, MODE_XBOX, MODE_SW_HORI };   // back4+B/X/Y -> these modes (A always STEAM)
bool    g_persistMode = false;
uint8_t g_bootMode = 0xFF;

bool          g_debugCdcThisBoot = false;
static uint8_t g_debugCdc = 0;   // persisted one-shot arm, stored in Cfg.rsvd0 (1 = keep CDC for the next boot)

int     g_mDiv = 64, g_mFric = 94;
uint8_t g_abSwap = 0;
uint8_t g_back[4] = {5,6,7,8};   // L4->LB R4->RB L5->L3 R5->R3 (see codeToXB)

// poll rate is fixed. Faster than the controller can refresh wastes airtime; slower adds latency. Any rate
// persisted by an older build is ignored and overwritten with the default on boot (see loadCfg).
const uint32_t g_pollUs = POLL_US_DEFAULT;

#define CFG_FILE "/cfg.bin"
#define CFG_MAGIC 0xC7   // bumped (chordBtn[3]): old cfg ignored -> clean defaults on first boot
struct Cfg { uint8_t magic, mode, mDiv, mFric, rsvd0, abSwap, back[4], pollU100, persistMode, bootMode, chordBtn[3]; };  // rsvd0 = ex-padSmooth, now the one-shot debug-CDC arm

void saveCfg(){
  Cfg c={CFG_MAGIC,g_usbMode,(uint8_t)g_mDiv,(uint8_t)g_mFric,g_debugCdc,g_abSwap,
         {g_back[0],g_back[1],g_back[2],g_back[3]},(uint8_t)(g_pollUs/100),(uint8_t)(g_persistMode?1:0),g_bootMode,
         {g_chordBtn[0],g_chordBtn[1],g_chordBtn[2]}};
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
      static const uint8_t CHORD_DEF[3]={MODE_LIZARD,MODE_XBOX,MODE_SW_HORI};
      for(int i=0;i<3;i++) g_chordBtn[i]=modeValid(c.chordBtn[i])?c.chordBtn[i]:CHORD_DEF[i];
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
