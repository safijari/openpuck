#include "webusb_config.h"
#include "config.h"
#include "bonds.h"
#include "rf_link.h"
#include "haptics.h"
#include "puck_hid.h"
#include "build_info.h"
#include <Arduino.h>
#include <string.h>

Adafruit_USBD_WebUSB usb_web;

// blob payload = [ver=2][mode][mDiv][mFric][rsvd=0][abSwap][back0..3][connSlot(0xFF=none)][linkUp]
//                [f1ps_lo][f1ps_hi][pollU100][newps_lo][newps_hi][e7b][relayOp][relaySub][fwdNewOnly]
//                [qos][persistMode][chordBtn B][chordBtn X][chordBtn Y][pollsps_lo][pollsps_hi]
//                [loopPeriod_lo][loopPeriod_hi][loopWorstIdx][loopWorstUs_lo][loopWorstUs_hi]
//                [pollPeriod_lo][pollPeriod_hi][logEnabled][battery%][rssi|dBm|]
//                [gitDirty][gitHash 12B ASCII, NUL-padded][buzzFlood]
#define WB_PAYLEN 52
static void webusbSendBlob(){
  if(!usb_web.connected()) return;
  bool up = (g_connSlot>=0 && (millis()-g_connReplyMs) < 300);
  uint8_t p[2+WB_PAYLEN];
  p[0]=0xA5; p[1]=WB_PAYLEN;
  p[2]=5;                          // protocol version (5 = +buzzFlood at [53]; 4 = git provenance; 3 = battery/rssi)
  p[3]=g_usbMode; p[4]=(uint8_t)g_mDiv; p[5]=(uint8_t)g_mFric; p[6]=0 /*rsvd: ex-padSmooth*/; p[7]=g_abSwap;
  p[8]=g_back[0]; p[9]=g_back[1]; p[10]=g_back[2]; p[11]=g_back[3];
  p[12]=(g_connSlot>=0)?(uint8_t)g_connSlot:0xFF;
  p[13]=up?1:0;
  p[14]=(uint8_t)g_f1ps; p[15]=(uint8_t)(g_f1ps>>8);
  p[16]=(uint8_t)(g_pollUs/100);
  p[17]=(uint8_t)g_newps; p[18]=(uint8_t)(g_newps>>8);
  p[19]=g_e7b;
  p[20]=g_relayOp; p[21]=g_relaySub; p[22]=g_fwdNewOnly; p[23]=g_qos; p[24]=g_persistMode?1:0;
  p[25]=g_chordBtn[0]; p[26]=g_chordBtn[1]; p[27]=g_chordBtn[2];
  p[28]=(uint8_t)g_pollsps; p[29]=(uint8_t)(g_pollsps>>8);   // poll TX rate (vs delivered/new -> starvation vs reply-loss)
  p[30]=(uint8_t)g_loopPeriodUs; p[31]=(uint8_t)(g_loopPeriodUs>>8);   // loop diag: avg iteration time + slowest section
  p[32]=g_loopWorst; p[33]=(uint8_t)g_loopWorstUs; p[34]=(uint8_t)(g_loopWorstUs>>8);
  p[35]=(uint8_t)g_pollPeriodUs; p[36]=(uint8_t)(g_pollPeriodUs>>8);   // measured actual poll period (vs intended 4000)
  p[37]=OPK_LOG;                                                       // logging build? panel shows/hides its log UI
  p[38]=g_battery;                                                     // controller battery % (report 0x43); 0=unknown
  p[39]=g_linkRssi;                                                    // RAW signal strength |dBm| (0=no sample)
  // Build provenance: which git commit this firmware was built from, and whether the tree was dirty. Lets the
  // panel confirm exactly what's flashed. Injected at build time (build_info.h / gen_version.sh); "unknown" if
  // the version header wasn't generated.
  p[40]=OPK_GIT_DIRTY ? 1 : 0;
  memset(&p[41],0,12);                                                 // 12B ASCII git hash, NUL-padded
  { const char* h=OPK_GIT_HASH; for(uint8_t i=0;i<12 && h[i];i++) p[41+i]=(uint8_t)h[i]; }
  p[53]=g_buzzFlood?1:0;                                               // connect-time buzz-clear flood enabled?
  usb_web.write(p,sizeof p); usb_web.flush();
}
#if OPK_LOG
// Stream the capture ring (haptics / relayed host commands) to the panel as 0xA6 frames, so the browser can
// pull it with no CDC. Frame formats:
//   entry: [0xA6][L][T=1][age u32 LE][slot][rid][n][bytes n]   (L = 8 + n)
//   end:   [0xA6][1][T=0]
static void webusbCapFrame(uint32_t ms,uint8_t slot,uint8_t rid,uint8_t nb,const uint8_t* b){
  if(nb>16) nb=16;
  uint8_t f[2+9+16]; uint8_t L=(uint8_t)(8+nb);
  f[0]=0xA6; f[1]=L; f[2]=1;
  f[3]=(uint8_t)ms; f[4]=(uint8_t)(ms>>8); f[5]=(uint8_t)(ms>>16); f[6]=(uint8_t)(ms>>24);  // absolute log time
  f[7]=slot; f[8]=rid; f[9]=nb; memcpy(f+10,b,nb);
  usb_web.write(f,(uint16_t)(2+L)); usb_web.flush();
}
static void webusbCapEnd(){ uint8_t e[3]={0xA6,1,0}; usb_web.write(e,3); usb_web.flush(); }
// 0x06: drain -- entries since the last drain (or since the 0x05 start/rewind). The panel polls this on a
// timer and accumulates, so a dump-from-boot of the whole ring streams over many polls without blocking the
// loop. A per-call budget bounds how long one poll spends here.
static void webusbDrainCapture(){
  if(!usb_web.connected()) return;
  uint32_t ms=0; uint8_t slot=0,rid=0,nb=0,b[16]; uint16_t budget=128;
  while(budget-- && hapLogPull(&ms,&slot,&rid,&nb,b)) webusbCapFrame(ms,slot,rid,nb,b);
  webusbCapEnd();
}
#endif // OPK_LOG
void webusbPoll(){
  static uint8_t buf[16]; static uint8_t n=0;
  while(usb_web.available()){
    int c=usb_web.read(); if(c<0) break;
    if(n<sizeof buf) buf[n++]=(uint8_t)c;
    // process complete commands from the front of buf
    for(;;){
      if(n==0) break;
      uint8_t op=buf[0]; uint8_t need = (op==0x02)?3 : (op==0x03||op==0x05)?2 : (op==0x09||op==0x0A)?4 : 1;   // 0x05 carries a value byte; 0x09 carries R,G,B; 0x0A carries a 3-byte magic
      if(op<0x01 || op>0x0A){ // resync: drop one byte
        memmove(buf,buf+1,--n); continue;
      }
      if(n<need) break;                      // wait for more bytes
      if(op==0x01){ webusbSendBlob(); }
#if OPK_LOG
      else if(op==0x05){ hapLogResetDrain(buf[1]!=0); }   // rewind drain: buf[1]=1 from boot (whole ring), 0 from now
      else if(op==0x06){ webusbDrainCapture(); }          // drain entries since the rewind (panel polls this)
#endif
      else if(op==0x07){ hapticArmBuzzFlood(); hapticReinit(); }   // (re)arm the 30s 10Hz buzz-clear flood; fire one now for instant feedback
      else if(op==0x08){ hapticSendShutdown(); }                   // TEST: trigger the controller power-off attempt (same path host-suspend uses)
      else if(op==0x0A){                                           // FULL factory wipe: erase cfg.bin + bonds.bin, reboot to clean defaults.
        // Guarded by a 3-byte magic ("ERS") so a stray/corrupt byte can never trigger it; the panel double-
        // confirms before sending. Irreversible -- the controller must be re-paired afterwards.
        if(buf[1]==0x45 && buf[2]==0x52 && buf[3]==0x53){ usb_web.flush(); factoryErase(); delay(40); NVIC_SystemReset(); }
      }
      else if(op==0x09){                                           // set LED color: feature-0x01 cmd 0xC1, 16-byte RGBW payload (from led-set capture)
        uint8_t led[16]={buf[1],buf[2],buf[3],0xff, 0x03,0x09,0x05, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};   // [R][G][B][W=ff][03 09 05][ff x9] -- captured frame, RGB substituted
        relayEnqueue(0xC1, led, sizeof led);
      }
      else if(op==0x02){
        uint8_t f=buf[1], v=buf[2];
        bool persist=true;   // every settable field persists (poll rate is no longer settable)
        switch(f){
          case 1: g_mDiv = v<4?4:v; break;
          case 2: g_mFric = v>99?99:v; break;
          // case 3 (padSmooth) removed -- Steam-mode pad coords are forwarded raw; Steam does its own smoothing.
          case 4: g_abSwap = v?1:0; break;
          case 5: case 6: case 7: case 8: g_back[f-5]=v; break;
          // case 9 (pollU100) removed -- poll rate is fixed at POLL_US_DEFAULT and no longer configurable.
          case 10: g_e7b = v?1:0; break;                       // E7 protocol-version B-byte (experimental v1 fast)
          case 11: g_relayOp = v; break;                       // haptic-relay opcode (buzz hunt)
          case 12: g_relaySub = v; break;                      // haptic-relay sub-type (buzz hunt)
          case 13: g_testHaptic = v?v:40; break;               // inject v test haptics (0->40)
          case 14: g_fwdNewOnly = v?1:0; break;                // Steam: forward only fresh reports (dedupe)
          case 15: g_qos = v?1:0; g_hopIdx=0; g_qosBad=0; g_qosCheckMs=millis(); break;  // QoS adaptive channel hopping
          case 16: g_persistMode = v?true:false; break;   // persist last mode across reboots (else always boot Steam)
          case 17: case 18: case 19: if(modeValid(v)) g_chordBtn[f-17]=v; break;   // back4+B/X/Y mode assignments
          case 20: armDebugCdcNextBoot(); usb_web.flush(); delay(40); NVIC_SystemReset(); break;  // reboot once WITH the CDC serial console (puck mode), then auto-revert
          case 21: g_buzzFlood = v?true:false; break;          // connect-time haptic buzz-clear flood (persisted; default off)
        }
        if(persist) saveCfg();
        webusbSendBlob();
      } else if(op==0x03){
        uint8_t m=buf[1]; if(modeValid(m) && !USBDevice.suspended()){ webusbSendBlob(); usb_web.flush(); saveMode(m); delay(40); NVIC_SystemReset(); }
      }
      memmove(buf,buf+need,n-need); n-=need;
    }
  }
}
