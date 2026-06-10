#include "webusb_config.h"
#include "config.h"
#include "bonds.h"
#include "rf_link.h"
#include "haptics.h"
#include "puck_hid.h"
#include <Arduino.h>
#include <string.h>

Adafruit_USBD_WebUSB usb_web;

// blob payload = [ver=2][mode][mDiv][mFric][rsvd=0][abSwap][back0..3][connSlot(0xFF=none)][linkUp]
//                [f1ps_lo][f1ps_hi][pollU100][newps_lo][newps_hi][e7b][relayOp][relaySub][fwdNewOnly]
//                [qos][persistMode][chordBtn B][chordBtn X][chordBtn Y]
#define WB_PAYLEN 26
static void webusbSendBlob(){
  if(!usb_web.connected()) return;
  bool up = (g_connSlot>=0 && (millis()-g_connReplyMs) < 300);
  uint8_t p[2+WB_PAYLEN];
  p[0]=0xA5; p[1]=WB_PAYLEN;
  p[2]=2;                          // protocol version (2 = chordBtn[3] in blob)
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
  usb_web.write(p,sizeof p); usb_web.flush();
}
void webusbPoll(){
  static uint8_t buf[16]; static uint8_t n=0;
  while(usb_web.available()){
    int c=usb_web.read(); if(c<0) break;
    if(n<sizeof buf) buf[n++]=(uint8_t)c;
    // process complete commands from the front of buf
    for(;;){
      if(n==0) break;
      uint8_t op=buf[0]; uint8_t need = (op==0x02)?3 : (op==0x03)?2 : (op==0x01)?1 : 1;
      if(op!=0x01 && op!=0x02 && op!=0x03){ // resync: drop one byte
        memmove(buf,buf+1,--n); continue;
      }
      if(n<need) break;                      // wait for more bytes
      if(op==0x01){ webusbSendBlob(); }
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
