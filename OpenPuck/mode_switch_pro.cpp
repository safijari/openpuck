#include "mode_switch_pro.h"
#include "triton.h"
#include "config.h"
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

SwitchProController g_switchPro;

static const uint8_t SWPRO_HID_DESC[]={
  0x05,0x01,0x15,0x00,0x09,0x04,0xA1,0x01,0x85,0x30,0x05,0x01,0x05,0x09,0x19,0x01,
  0x29,0x0A,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x0A,0x55,0x00,0x65,0x00,0x81,0x02,
  0x05,0x09,0x19,0x0B,0x29,0x0E,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,0x02,
  0x75,0x01,0x95,0x02,0x81,0x03,0x0B,0x01,0x00,0x01,0x00,0xA1,0x00,0x0B,0x30,0x00,
  0x01,0x00,0x0B,0x31,0x00,0x01,0x00,0x0B,0x32,0x00,0x01,0x00,0x0B,0x35,0x00,0x01,
  0x00,0x15,0x00,0x27,0xFF,0xFF,0x00,0x00,0x75,0x10,0x95,0x04,0x81,0x02,0xC0,0x0B,
  0x39,0x00,0x01,0x00,0x15,0x00,0x25,0x07,0x35,0x00,0x46,0x3B,0x01,0x65,0x14,0x75,
  0x04,0x95,0x01,0x81,0x42,0x05,0x09,0x19,0x0F,0x29,0x12,0x15,0x00,0x25,0x01,0x75,
  0x01,0x95,0x04,0x81,0x02,0x75,0x08,0x95,0x34,0x81,0x03,0x06,0x00,0xFF,0x85,0x21,
  0x09,0x01,0x75,0x08,0x95,0x3F,0x81,0x03,0x85,0x81,0x09,0x02,0x75,0x08,0x95,0x3F,
  0x81,0x03,0x85,0x01,0x09,0x03,0x75,0x08,0x95,0x3F,0x91,0x83,0x85,0x10,0x09,0x04,
  0x75,0x08,0x95,0x3F,0x91,0x83,0x85,0x80,0x09,0x05,0x75,0x08,0x95,0x3F,0x91,0x83,
  0x85,0x82,0x09,0x06,0x75,0x08,0x95,0x3F,0x91,0x83,0xC0
};
#define JC_BTN_Y      (1u<<0)
#define JC_BTN_X      (1u<<1)
#define JC_BTN_B      (1u<<2)
#define JC_BTN_A      (1u<<3)
#define JC_BTN_R      (1u<<6)
#define JC_BTN_ZR     (1u<<7)
#define JC_BTN_MINUS  (1u<<8)
#define JC_BTN_PLUS   (1u<<9)
#define JC_BTN_RSTICK (1u<<10)
#define JC_BTN_LSTICK (1u<<11)
#define JC_BTN_HOME   (1u<<12)
#define JC_BTN_DOWN   (1u<<16)
#define JC_BTN_UP     (1u<<17)
#define JC_BTN_RIGHT  (1u<<18)
#define JC_BTN_LEFT   (1u<<19)
#define JC_BTN_L      (1u<<22)
#define JC_BTN_ZL     (1u<<23)
static uint32_t codeToJc(uint8_t c, uint32_t fA,uint32_t fB,uint32_t fX,uint32_t fY){
  switch(c){ case 1:return fA; case 2:return fB; case 3:return fX; case 4:return fY;
    case 5:return JC_BTN_L; case 6:return JC_BTN_R; case 7:return JC_BTN_LSTICK; case 8:return JC_BTN_RSTICK;
    case 9:return JC_BTN_MINUS; case 10:return JC_BTN_PLUS; case 11:return JC_BTN_HOME; default:return 0; }
}
static unsigned long g_swProLastMs=0;
static Adafruit_USBD_HID g_swPro;
static uint8_t g_swProReportMode=0;   // 0 until the host's subcommand 0x03 selects 0x30 -> THEN we stream input
static const uint8_t JC_MAC[6]={0x7C,0xBB,0x8A,0x00,0x00,0x01};   // synthetic but stable controller MAC
static int jcStick12(int16_t v, bool inv){   // steam int16 (center 0) -> 12-bit (center 0x800), clamped
  int a = 2048 + (inv ? -((int)v>>4) : ((int)v>>4));
  return a<0?0:(a>4095?4095:a);
}
static void jcPackStick(uint8_t s[3], int16_t x, int16_t y){   // pack two 12-bit axes into 3 bytes (both normal; Switch Y polarity is opposite DS4)
  int X=jcStick12(x,false), Y=jcStick12(y,false);
  s[0]=(uint8_t)(X&0xFF);
  s[1]=(uint8_t)(((Y&0x0F)<<4)|((X>>8)&0x0F));
  s[2]=(uint8_t)((Y>>4)&0xFF);
}
// Standard input-report prefix [0..11] (timer, battery/conn, 3 button bytes, both packed sticks, vibrator),
// shared by the streamed 0x30 report and the 0x21 subcommand-reply reports the host reads during init.
static uint8_t g_jcTimer=0;
static void jcInputPrefix(uint8_t* out){
  uint32_t b=g_in.buttons;
  if((b&CHORD_BACK4)==CHORD_BACK4) b&=~(uint32_t)(TB_A|TB_B|TB_X|TB_Y);
  uint32_t fA=g_abSwap?JC_BTN_B:JC_BTN_A, fB=g_abSwap?JC_BTN_A:JC_BTN_B;
  uint32_t fX=g_abSwap?JC_BTN_Y:JC_BTN_X, fY=g_abSwap?JC_BTN_X:JC_BTN_Y;
  uint32_t jc=0;
  if(b&TB_Y)jc|=fY; if(b&TB_B)jc|=fB; if(b&TB_A)jc|=fA; if(b&TB_X)jc|=fX;
  if(b&TB_LB)jc|=JC_BTN_L; if(b&TB_RB)jc|=JC_BTN_R;
  if((g_in.lt>=SW_TRIG_ON)||(b&0x8000000u))jc|=JC_BTN_ZL; if((g_in.rt>=SW_TRIG_ON)||(b&0x800000u))jc|=JC_BTN_ZR;
  if(b&TB_VIEW)jc|=JC_BTN_PLUS; if(b&TB_MENU)jc|=JC_BTN_MINUS;
  if(b&TB_L3)jc|=JC_BTN_LSTICK; if(b&TB_R3)jc|=JC_BTN_RSTICK;
  if(b&TB_STEAM)jc|=JC_BTN_HOME;
  if(b&TB_DDN)jc|=JC_BTN_DOWN; if(b&TB_DUP)jc|=JC_BTN_UP;
  if(b&TB_DRT)jc|=JC_BTN_RIGHT; if(b&TB_DLF)jc|=JC_BTN_LEFT;
  if(b&TB_L4)jc|=codeToJc(g_back[0],fA,fB,fX,fY); if(b&TB_R4)jc|=codeToJc(g_back[1],fA,fB,fX,fY);
  if(b&TB_L5)jc|=codeToJc(g_back[2],fA,fB,fX,fY); if(b&TB_R5)jc|=codeToJc(g_back[3],fA,fB,fX,fY);
  out[0]=g_jcTimer++;
  out[1]=0x81;   // full battery + USB powered (hid-nintendo bat_con)
  out[2]=(uint8_t)(jc); out[3]=(uint8_t)(jc>>8); out[4]=(uint8_t)(jc>>16);
  jcPackStick(out+5, g_in.lx, g_in.ly);
  jcPackStick(out+8, g_in.rx, g_in.ry);
  out[11]=0;
}
static void switchProBuild(uint8_t out[63]){
  memset(out,0,63);
  jcInputPrefix(out);
  for(int k=0;k<3;k++){
    int o=12+k*12;
    out[o+0]=g_in.ax&0xFF; out[o+1]=g_in.ax>>8;
    out[o+2]=g_in.ay&0xFF; out[o+3]=g_in.ay>>8;
    out[o+4]=g_in.az&0xFF; out[o+5]=g_in.az>>8;
    out[o+6]=g_in.gx&0xFF; out[o+7]=g_in.gx>>8;
    out[o+8]=g_in.gy&0xFF; out[o+9]=g_in.gy>>8;
    out[o+10]=g_in.gz&0xFF; out[o+11]=g_in.gz>>8;
  }
}
// --- Canonical factory SPI dumps the host reads for calibration. Neutral IMU + centered sticks so a fresh
// "device" calibrates sane; user-cal regions (0x80xx) read 0xFF so the host falls back to these factory blocks.
static const uint8_t SPI_IMU_CAL[24]={0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x40,0x00,0x40,0x00,0x40,
                                      0x00,0x00,0x00,0x00,0x00,0x00, 0x3B,0x34,0x3B,0x34,0x3B,0x34};
static const uint8_t SPI_PARAMS1[24]={0x50,0xFD,0x00,0x00,0xC6,0x0F, 0x0F,0x30,0x61,0x96,0x30,0xF3,
                                      0xD4,0x14,0x54,0x41,0x15,0x54, 0xC7,0x79,0x9C,0x33,0x36,0x63};
static const uint8_t SPI_PARAMS2[18]={0x0F,0x30,0x61,0x96,0x30,0xF3, 0xD4,0x14,0x54,0x41,0x15,0x54,
                                      0xC7,0x79,0x9C,0x33,0x36,0x63};
static const uint8_t SPI_COLOR[13]={0x32,0x32,0x32, 0xE6,0xE6,0xE6, 0x32,0x32,0x32, 0x32,0x32,0x32, 0xFF};
static uint8_t g_spiStickCal[18];   // built at boot: left[9]+right[9] factory stick calibration (packed 12-bit)
static void jcPack12(uint8_t* o9, const uint16_t v[6]){   // pack 6 12-bit values into 9 bytes (Switch stick-cal format)
  o9[0]=v[0]&0xFF;                       o9[1]=((v[1]&0x0F)<<4)|((v[0]>>8)&0x0F); o9[2]=(v[1]>>4)&0xFF;
  o9[3]=v[2]&0xFF;                       o9[4]=((v[3]&0x0F)<<4)|((v[2]>>8)&0x0F); o9[5]=(v[3]>>4)&0xFF;
  o9[6]=v[4]&0xFF;                       o9[7]=((v[5]&0x0F)<<4)|((v[4]>>8)&0x0F); o9[8]=(v[5]>>4)&0xFF;
}
static void jcBuildStickCal(){
  const uint16_t C=2048, R=1800;   // center, +/- range per axis
  uint16_t L[6]={R,R, C,C, R,R};   // left  order: max(x,y), center(x,y), min(x,y)
  uint16_t Rr[6]={C,C, R,R, R,R};  // right order: center(x,y), min(x,y), max(x,y)
  jcPack12(&g_spiStickCal[0], L); jcPack12(&g_spiStickCal[9], Rr);
}
static void spiRead(uint32_t addr, uint8_t len, uint8_t* dst){
  for(uint8_t i=0;i<len;i++){
    uint32_t a=addr+i; uint8_t v=0xFF;
    if     (a>=0x6020 && a<0x6020+24) v=SPI_IMU_CAL[a-0x6020];
    else if(a>=0x603D && a<0x603D+18) v=g_spiStickCal[a-0x603D];
    else if(a>=0x6050 && a<0x6050+13) v=SPI_COLOR[a-0x6050];
    else if(a>=0x6080 && a<0x6080+24) v=SPI_PARAMS1[a-0x6080];
    else if(a>=0x6098 && a<0x6098+18) v=SPI_PARAMS2[a-0x6098];
    dst[i]=v;   // unmapped (incl. user-cal 0x80xx) -> 0xFF = "blank flash" -> host uses factory blocks above
  }
}
// Reply FIFO: the host's handshake/subcommand reports arrive in the USB ISR (jcSet); we enqueue the canonical
// 0x81/0x21 reply and drain it from task() where sendReport is safe. SPSC ring, volatile indices. Every reply
// is sent as a full 63-byte report (64 with the id): macOS IOHIDManager silently DROPS any input report shorter
// than the descriptor-declared length (0x3F), so short 0x81/0x21 replies never reached the host.
#define JC_REPLEN 63
struct JcRep { uint8_t rid; uint8_t data[JC_REPLEN]; };
#define JCQ_N 8
static volatile uint8_t g_jcQh=0, g_jcQt=0;
static JcRep g_jcQ[JCQ_N];
static void jcEnq(uint8_t rid, const uint8_t* d, uint8_t len){
  uint8_t nt=(uint8_t)((g_jcQt+1)%JCQ_N);
  if(nt==g_jcQh) return;                 // full -> drop; the host re-requests on timeout
  if(len>JC_REPLEN) len=JC_REPLEN;
  g_jcQ[g_jcQt].rid=rid;
  memset(g_jcQ[g_jcQt].data,0,JC_REPLEN); memcpy(g_jcQ[g_jcQt].data,d,len);   // zero-pad to full report length
  g_jcQt=nt;
}
// Build the 0x21 reply (standard input prefix + ACK + echoed subcommand id + reply data) for a report-0x01 subcommand.
static void jcSubcmd(uint8_t sub, const uint8_t* args, uint16_t alen){
  uint8_t p[JC_REPLEN]; memset(p,0,sizeof p);
  jcInputPrefix(p);
  p[13]=sub;
  switch(sub){
    case 0x02:                         // request device info
      p[12]=0x82;
      p[14]=0x04; p[15]=0x21;          // firmware version (4.33)
      p[16]=0x03;                      // controller type: Pro Controller
      p[17]=0x02;                      // fixed
      memcpy(&p[18], JC_MAC, 6);
      p[24]=0x01;                      // colors stored in SPI (0x6050)
      p[25]=0x02;                      // fixed
      break;
    case 0x10: {                       // SPI flash read -> echo [addr][len] then the data
      if(alen<5){ p[12]=0x80; break; }
      uint32_t a=(uint32_t)args[0]|((uint32_t)args[1]<<8)|((uint32_t)args[2]<<16)|((uint32_t)args[3]<<24);
      uint8_t rl=args[4]; if(rl>0x1D) rl=0x1D;
      p[12]=0x90;
      p[14]=args[0]; p[15]=args[1]; p[16]=args[2]; p[17]=args[3]; p[18]=rl;
      spiRead(a, rl, &p[19]);
      break; }
    case 0x03:                         // set input report mode (0x30 = standard full) -> begin streaming
      if(alen>=1 && args[0]==0x30) g_swProReportMode=0x30;
      p[12]=0x80; break;
    case 0x04: p[12]=0x83; break;      // trigger buttons elapsed time
    case 0x21: p[12]=0xA0; break;      // set NFC/IR config
    default:   p[12]=0x80; break;      // 0x06/0x08/0x30/0x38/0x40/0x41/0x48/... generic positive ACK
  }
  jcEnq(0x21, p, JC_REPLEN);
}
// USB-ISR callback for the Pro Controller's OUT endpoint. Data arrives via the interrupt OUT endpoint, where
// this TinyUSB core passes report_id=0 and leaves the real report id in buffer[0] (it does NOT strip it). Also
// accept the control SET_REPORT shape (id already in `rid`) for safety. report 0x80 = USB handshake; 0x01 = subcommand.
static void jcSet(uint8_t rid, hid_report_type_t type, uint8_t const* b, uint16_t n){
  if(type!=HID_REPORT_TYPE_OUTPUT || n<1) return;
  uint8_t id; const uint8_t* p; uint16_t pn;
  if(rid==0){ id=b[0]; p=b+1; pn=(uint16_t)(n-1); }   // EP-OUT: id is the first payload byte
  else      { id=rid;  p=b;   pn=n;               }   // control SET_REPORT: id already split out
  if(id==0x80){
    if(pn<1) return;
    if(p[0]==0x01){ uint8_t d[9]={0x01,0x00,0x03,JC_MAC[0],JC_MAC[1],JC_MAC[2],JC_MAC[3],JC_MAC[4],JC_MAC[5]}; jcEnq(0x81,d,9); } // device type + MAC
    else if(p[0]==0x02){ uint8_t d[1]={0x02}; jcEnq(0x81,d,1); }   // handshake
    else if(p[0]==0x03){ uint8_t d[1]={0x03}; jcEnq(0x81,d,1); }   // set baudrate
    // 0x04 force-USB / 0x05 enable-timeout / 0x06 reset: no reply expected
    return;
  }
  if(id==0x01){                        // [timer][rumble x8][subcmd][args...]
    if(pn<10) return;
    jcSubcmd(p[9], p+10, (pn>10)?(uint16_t)(pn-10):0);
  }
  // report 0x10 (rumble only) / 0x82: ignored
}

void SwitchProController::begin(){
  USBDevice.setID(0x057E, 0x2009);
  USBDevice.setDeviceVersion(0x0210);
  USBDevice.setManufacturerDescriptor("Nintendo Co., Ltd.");
  USBDevice.setProductDescriptor("Pro Controller");
  jcBuildStickCal();
  g_swPro.enableOutEndpoint(true);
  g_swPro.setReportCallback(NULL, jcSet);   // answer the Nintendo USB handshake + subcommands (else Steam never binds it)
  g_swPro.setReportDescriptor(SWPRO_HID_DESC, sizeof SWPRO_HID_DESC);
  g_swPro.setPollInterval(4);
  g_swPro.begin();
}
void SwitchProController::task(){
  if(!g_swPro.ready()) return;
  if(g_jcQh!=g_jcQt){                                   // drain handshake/subcommand replies first (ordered)
    JcRep* r=&g_jcQ[g_jcQh];
    if(g_swPro.sendReport(r->rid, r->data, JC_REPLEN)) g_jcQh=(uint8_t)((g_jcQh+1)%JCQ_N);
    return;                                             // one report per call; the rest next loop
  }
  if(g_swProReportMode!=0x30) return;                   // not until the host has finished init + selected 0x30
  if(millis()-g_swProLastMs<USB_STREAM_MS) return; g_swProLastMs=millis();
  uint8_t p[63]; switchProBuild(p);
  g_swPro.sendReport(0x30, p, sizeof p);
}
