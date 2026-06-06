// OpenPuck.ino -- Steam Controller 2 ("Triton") puck reimplementation for an nRF52840.
//
// This sketch impersonates the Valve puck over USB, maintains puck-style bond slots, speaks the
// reverse-engineered RF protocol to the controller, and re-enumerates into Steam, Xbox, or Switch
// personalities. The real puck exposes four separate HID control interfaces; interface N owns bond
// slot N, so this sketch does the same. Build with -DCFG_TUD_HID=4 (the Adafruit nRF port defaults to 2).
//
// USB presentations: Steam = the genuine 4-slot Valve puck (28DE:1304) with the feature-report command
// channel (0x83/0xAE/0xB4/0xA3/0xA2/0xAD); desktop keyboard+mouse "lizard" is automatic when Steam is
// closed. Xbox = a clean XInput controller (045E:028E) + right-pad mouse. Switch = a HORIPAD pad (0F0D:0092).
// Xbox/Switch omit CDC/WebUSB so strict hosts bind them without vendor/config side effects.
//
// Protocol reference: docs/PROTOCOL.md.  Bond record = [8 uuid][16 serial].

#include <Adafruit_TinyUSB.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
// custom class-driver API (usbd_class_driver_t, usbd_edpt_*) for the XInput interface
extern "C" {
#include "device/usbd_pvt.h"
}
// Bounded radio-disable wait: NEVER spin forever. A wedged RADIO peripheral would otherwise hang the whole
// main loop (and USB stops being serviced -> device "dies" until replug). On timeout we bail and continue;
// the next rfConfig re-inits the radio.
#define RWAIT_DISABLED() do{ uint32_t _w=micros(); while(!NRF_RADIO->EVENTS_DISABLED && (uint32_t)(micros()-_w)<3000){} }while(0)
using namespace Adafruit_LittleFS_Namespace;

#if CFG_TUD_HID < 4
#error "build with -DCFG_TUD_HID=4 (extra_flags): up to 4 HID interfaces per mode"
#endif

// ---- cloned puck HID report descriptor (verbatim): mouse(0x40)+keyboard(0x41)+vendor(FF00) with
//      the 63-byte FEATURE command reports on report id 1/2. Each of the 4 interfaces uses this. ----
static const uint8_t PUCK_HID_DESC[] = {
  0x05,0x01,0x09,0x02,0xA1,0x01,0x85,0x40,0x09,0x01,0xA1,0x00,0x05,0x09,0x19,0x01,
  0x29,0x02,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x02,0x81,0x02,0x75,0x06,0x95,0x01,
  0x81,0x01,0x05,0x01,0x09,0x30,0x09,0x31,0x15,0x81,0x25,0x7F,0x75,0x08,0x95,0x02,
  0x81,0x06,0x95,0x01,0x09,0x38,0x81,0x06,0x05,0x0C,0x0A,0x38,0x02,0x95,0x01,0x81,
  0x06,0xC0,0xC0,0x05,0x01,0x09,0x06,0xA1,0x01,0x85,0x41,0x05,0x07,0x19,0xE0,0x29,
  0xE7,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x08,0x81,0x02,0x81,0x01,0x19,0x00,0x29,
  0x65,0x15,0x00,0x25,0x65,0x75,0x08,0x95,0x06,0x81,0x00,0xC0,0x06,0x00,0xFF,0x09,
  0x01,0xA1,0x01,0x85,0x42,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x35,0x09,0x42,
  0x81,0x02,0x85,0x44,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x05,0x09,0x44,0x81,
  0x02,0x85,0x79,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x01,0x09,0x79,0x81,0x02,
  0x85,0x43,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x0E,0x09,0x43,0x81,0x02,0x85,
  0x7B,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x0C,0x09,0x7B,0x81,0x02,0x85,0x45,
  0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x2D,0x09,0x45,0x81,0x02,0x85,0x80,0x15,
  0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x09,0x09,0x80,0x91,0x02,0x85,0x81,0x15,0x00,
  0x26,0xFF,0x00,0x75,0x08,0x95,0x07,0x09,0x81,0x91,0x02,0x85,0x82,0x15,0x00,0x26,
  0xFF,0x00,0x75,0x08,0x95,0x03,0x09,0x82,0x91,0x02,0x85,0x83,0x15,0x00,0x26,0xFF,
  0x00,0x75,0x08,0x95,0x09,0x09,0x83,0x91,0x02,0x85,0x84,0x15,0x00,0x26,0xFF,0x00,
  0x75,0x08,0x95,0x08,0x09,0x84,0x91,0x02,0x85,0x85,0x15,0x00,0x26,0xFF,0x00,0x75,
  0x08,0x95,0x03,0x09,0x85,0x91,0x02,0x85,0x86,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,
  0x95,0x03,0x09,0x86,0x91,0x02,0x85,0x87,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,
  0x3F,0x09,0x87,0x91,0x02,0x85,0x89,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x3F,
  0x09,0x89,0x91,0x02,0x85,0x88,0x15,0x00,0x26,0xFF,0x00,0x75,0x08,0x95,0x3F,0x09,
  0x88,0x91,0x02,0x85,0x01,0x95,0x3F,0x09,0x01,0xB1,0x02,0x85,0x02,0x95,0x3F,0x09,
  0x01,0xB1,0x02,0xC0
};

#define NSLOT 4
Adafruit_USBD_HID hid[NSLOT];
// XBOX MODE = a MUTUALLY-EXCLUSIVE USB presentation: the device enumerates as EITHER the 4 puck slot
// interfaces (Steam mode) OR an XInput controller + a mouse (Xbox mode) — never both. The mode is persisted to
// flash and a button combo (back4+X -> xbox, back4+A -> steam) saves it and reboots, so each boot enumerates
// the right interface set. RF (beacon/poll/bonds) is identical in both modes; only the USB side differs.
static const uint8_t MOUSE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
Adafruit_USBD_HID g_mouse;   // Xbox-mode mouse interface (right trackpad). Steam-mode lizard reuses the puck's own 0x40 mouse report instead.
static uint8_t g_usbMode = 0;   // 0=STEAM(puck; auto-lizard when Steam closed) 1=XBOX(xinput+mouse) 2=SWITCH(HORIPAD); loaded from flash at boot
static bool    g_xbox = false;   // == (g_usbMode != 0); kept for the existing puck-vs-other checks (true for xbox/switch)
// Mode persistence policy: by DEFAULT every fresh power-on/reconnect lands in STEAM mode (0). An explicit
// mode switch still works for the session via a ONE-SHOT bootMode (honored once, then cleared, so the next
// cold boot reverts to Steam). The WebUI "persist last mode" toggle (g_persistMode) instead remembers the
// last selected mode across reboots.
static bool    g_persistMode = false;   // false (default) = always boot Steam; true = boot into last mode
static uint8_t g_bootMode = 0xFF;       // one-shot: boot into this mode once then clear (used when !persistMode + explicit switch)
volatile uint8_t g_rumble = 0;   // legacy XInput rumble strength from the host's OUT packet (0=off); drives the haptic relay when the XInput descriptor is used
volatile unsigned long g_rumbleMs = 0;   // millis of last rumble OUT packet (declared here — used by xi_xfer above the watchdog block); stuck-rumble watchdog
// persisted, runtime-tunable config (Cfg struct + load/saveCfg below; CDC M/F/D/W/B + WebUSB set these):
static int     g_mDiv = 64, g_mFric = 94, g_padSmooth = 35;  // xbox mouse sens/friction%, steam pad smoothing%
static uint8_t g_abSwap = 0;                 // 1 = swap A/B and X/Y (Nintendo face-button layout)
static uint8_t g_back[4] = {5,6,7,8};        // back paddles L4,R4,L5,R5 -> button codes (see codeToXB; 5=LB 6=RB 7=L3 8=R3)
static uint32_t g_pollUs = 800;              // RF poll cadence (us). LOCKED default: 800us is the measured peak (~706 polls/s -> ~400 new/s with PID cycling; below 800 = RX-turnaround-limited, above 800 declines). NOT persisted — WebUSB/CDC 'U' tune it for the session only, reverts to 800 on reboot.
                                             // but too-aggressive over-polls -> reply collapse during trackpad. Live-tunable.
static uint32_t g_rxWin = 1200;              // poll RX-window (us): max wait for the reply. This (not pollUs) caps poll rate
                                             // (~1e6/rxWin). Shorter=more polls/s but may miss the controller's DELAYED replies
                                             // during trackpad. Tunable 'r<us>'. The reply normally arrives fast; 1200 is the worst-case wait.
static uint8_t g_fwdNewOnly = 1;             // Steam mode: forward report 0x45 only when seq advanced (dedupe like the real puck).
                                             // Sending stale repeats makes Steam's trackpad smoothing stair-step. Toggle to A/B.
// QoS adaptive channel hopping: track link badness (crcfail+noRx) on the current channel; when it degrades,
// hop to the next clean candidate (controller follows via rfHopTo). Conservative hysteresis to avoid blip-storms.
static uint8_t  g_qos = 0;                   // 0=off (static g_sessCh), 1=auto-hop on degradation
static const uint8_t g_hopCand[] = {18,46,76,22,68};  // clean, spread channels (from the puck's RSSI/PER scan)
static uint8_t  g_hopIdx = 0;
static volatile uint16_t g_qosBad = 0;       // crcfail+noRx accumulated since last QoS check
static unsigned long g_qosCheckMs = 0, g_qosLastHopMs = 0;

// WebUSB config channel — a dedicated vendor interface present in every mode so the browser panel
// (copycat_config.html) can read/set the tunables above. Protocol + poll in webusbPoll() below.
// NOTE: no setLandingPage() — that would make Chrome pop a "device detected, open <url>" notification
// every plug-in. The interface works without it; the user just opens copycat_config.html themselves.
Adafruit_USBD_WebUSB usb_web;

// ===================== XInput (Xbox 360 wired) device — Xbox-mode gamepad =====================
// Real Xbox 360 pads are NOT HID: vendor interface class 0xFF/sub 0x5D/proto 0x01 + 20-byte XInput report.
// Custom TinyUSB class driver (registered via usbd_app_driver_get_cb) + an Adafruit_USBD_Interface subclass
// that emits the interface+0x21-blob+2 endpoints. Steam/SDL bind it as "Xbox 360 Controller" by VID/PID +
// the FF/5D/01 interface. (Verified against Adafruit nRF52 1.7.0 / TinyUSB 0.18.)  XInput button bits:
enum { XB_DUP=0x0001,XB_DDOWN=0x0002,XB_DLEFT=0x0004,XB_DRIGHT=0x0008, XB_START=0x0010,XB_BACK=0x0020,
       XB_L3=0x0040,XB_R3=0x0080, XB_LB=0x0100,XB_RB=0x0200,XB_GUIDE=0x0400, XB_A=0x1000,XB_B=0x2000,XB_X=0x4000,XB_Y=0x8000 };
#define XINPUT_DESC_LEN (9 + 17 + 7 + 7)   // interface(9)+vendor0x21(17)+IN ep(7)+OUT ep(7) = 40
static uint8_t g_xiItf=0xFF, g_xiEpIn=0, g_xiEpOut=0;
static uint8_t g_xiInBuf[32], g_xiOutBuf[32];
static void xi_init(void){}
static bool xi_deinit(void){return true;}
static void xi_reset(uint8_t rhport){(void)rhport; g_xiEpIn=g_xiEpOut=0;}
static uint16_t xi_open(uint8_t rhport, tusb_desc_interface_t const* itf, uint16_t max_len){
  if(!(itf->bInterfaceClass==0xFF && itf->bInterfaceSubClass==0x5D && itf->bInterfaceProtocol==0x01)) return 0;
  g_xiItf=itf->bInterfaceNumber;
  uint8_t const* p=(uint8_t const*)itf; uint8_t const* end=p+max_len; uint16_t used=itf->bLength; p+=itf->bLength;
  uint8_t opened=0;
  while(p<end && opened<itf->bNumEndpoints){
    uint8_t blen=p[0], btype=p[1];
    if(btype==TUSB_DESC_ENDPOINT){ tusb_desc_endpoint_t const* ep=(tusb_desc_endpoint_t const*)p;
      usbd_edpt_open(rhport,ep);
      if(tu_edpt_dir(ep->bEndpointAddress)==TUSB_DIR_IN) g_xiEpIn=ep->bEndpointAddress; else g_xiEpOut=ep->bEndpointAddress;
      opened++; }
    used+=blen; p+=blen;
  }
  if(g_xiEpOut) usbd_edpt_xfer(rhport,g_xiEpOut,g_xiOutBuf,sizeof g_xiOutBuf);  // arm OUT (rumble/LED)
  return used;
}
static bool xi_ctrl(uint8_t rhport,uint8_t stage,tusb_control_request_t const* req){(void)rhport;(void)req; return stage!=CONTROL_STAGE_SETUP;}
static bool xi_xfer(uint8_t rhport,uint8_t ep,xfer_result_t res,uint32_t n){(void)res;
  if(ep==g_xiEpOut){
    // XInput rumble packet: [00][08][00][bigMotor][smallMotor][00][00][00]; LED pkt is [01][03][led]
    if(n>=5 && g_xiOutBuf[0]==0x00 && g_xiOutBuf[1]==0x08){
      uint8_t big=g_xiOutBuf[3], sml=g_xiOutBuf[4]; g_rumble = big>sml?big:sml; g_rumbleMs = millis();   // stamp for the stuck-rumble watchdog
    }
    usbd_edpt_xfer(rhport,g_xiEpOut,g_xiOutBuf,sizeof g_xiOutBuf);    // re-arm OUT
  }
  return true; }
static const usbd_class_driver_t g_xiDriver = {
#if CFG_TUSB_DEBUG>=2
  .name="XINPUT",
#endif
  .init=xi_init, .deinit=xi_deinit, .reset=xi_reset, .open=xi_open,
  .control_xfer_cb=xi_ctrl, .xfer_cb=xi_xfer, .sof=NULL };
extern "C" const usbd_class_driver_t* usbd_app_driver_get_cb(uint8_t* count){ *count=1; return &g_xiDriver; }
class Adafruit_USBD_XInput : public Adafruit_USBD_Interface {
public:
  uint16_t getInterfaceDescriptor(uint8_t, uint8_t* buf, uint16_t bufsize) override {
    if(!buf) return XINPUT_DESC_LEN; if(bufsize<XINPUT_DESC_LEN) return 0;
    uint8_t itfnum=TinyUSBDevice.allocInterface(1);
    uint8_t epin=TinyUSBDevice.allocEndpoint(TUSB_DIR_IN), epout=TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
    const uint8_t t[XINPUT_DESC_LEN]={
      9,TUSB_DESC_INTERFACE,itfnum,0x00,0x02,0xFF,0x5D,0x01,_strid,
      0x11,0x21,0x00,0x01,0x01,0x25,epin,0x14,0x00,0x00,0x00,0x00,0x13,epout,0x08,0x00,0x00,
      7,TUSB_DESC_ENDPOINT,epin, TUSB_XFER_INTERRUPT,U16_TO_U8S_LE(0x20),1,   // bInterval 1ms (1000Hz) so the RF rate is the only limit
      7,TUSB_DESC_ENDPOINT,epout,TUSB_XFER_INTERRUPT,U16_TO_U8S_LE(0x20),8 };
    memcpy(buf,t,XINPUT_DESC_LEN); return XINPUT_DESC_LEN;
  }
  bool begin(){ return TinyUSBDevice.addInterface(*this); }
};
Adafruit_USBD_XInput g_xinput;
static void xinputSend(uint16_t buttons,uint8_t lt,uint8_t rt,int16_t lx,int16_t ly,int16_t rx,int16_t ry){
  if(!tud_mounted() || g_xiEpIn==0 || usbd_edpt_busy(0,g_xiEpIn)) return;
  uint8_t* r=g_xiInBuf;
  r[0]=0x00; r[1]=0x14; r[2]=buttons&0xFF; r[3]=buttons>>8; r[4]=lt; r[5]=rt;
  r[6]=lx&0xFF; r[7]=lx>>8; r[8]=ly&0xFF; r[9]=ly>>8; r[10]=rx&0xFF; r[11]=rx>>8; r[12]=ry&0xFF; r[13]=ry>>8;
  memset(r+14,0,6);
  if(usbd_edpt_claim(0,g_xiEpIn)){ if(!usbd_edpt_xfer(0,g_xiEpIn,r,20)) usbd_edpt_release(0,g_xiEpIn); }
}

// ---- identity: a UNIQUE serial from the FICR DEVICEID (never clashes with a real puck) ----
static char g_unit[16];   // "FXB99602xxxxx"
static char g_board[16];  // "MXB99602xxxxx"
static char g_usbSerial[18];   // per-mode USB serial (Steam uses g_unit; others get a suffix — see setup)
static const uint8_t ATTR83[] = {     // 0x83 attributes (product 0x1304 = puck)
  0x01,0x04,0x13,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x0A,0xF2,0xF9,0xD2,0x68,
  0x04,0x53,0xD0,0x18,0x6A,0x09,0x47,0x00,0x00,0x00 };

// ---- per-slot state (record = [8 uuid][16 serial]) + per-interface response staging ----
struct Slot { uint8_t rec[24]; bool used; uint8_t resp[63]; uint16_t resp_len; };
static Slot g_slot[NSLOT];
// connection tracking (defined here so the USB feature handler can report B4 connection state)
static int           g_connSlot = -1;    // bonded slot being polled (== the USB-connected slot)
static unsigned long g_connReplyMs = 0;  // millis of last RF reply (link-alive timestamp)
// host->controller relay: Steam writes feature report id 0x01 (raw passthrough) carrying a controller
// feature report [report-id][len][payload...] (e.g. 87 03 09 00 00 = settings: lizard-mode off). The real
// dongle forwards it to the controller as a SET sub-TLV in the E3 poll; the copycat must do the same.
static volatile bool g_relayPend = false;
static uint8_t       g_relayBuf[24];
static volatile uint8_t g_relayN = 0;
static uint8_t g_relayOp  = 0xE3;   // relay frame opcode (E3 poll)
static uint8_t g_relaySub = 0x05;   // relay sub-TLV type byte = SET (confirmed on hardware: E3 + [len][05][rid][payload] makes the controller act on it — haptics buzz + lizard-off lands). GET is 01.
static volatile uint8_t g_testHaptic = 0;   // 't<n>' injects n test haptics (output 0x82 [01 01 F7]) for the buzz hunt
// Haptic watchdog — stops the "stuck high-freq buzz" when the host's stop command is lost over RF.
// Xbox rumble is a persistent LEVEL with no keepalive: a dropped stop packet leaves g_rumble set and the
// relay re-buzzes at 40Hz forever (until a host event like the Steam button clears it). Steam glide haptics
// can likewise latch on the controller if Steam's stop relay is lost. We auto-release on host silence.
static unsigned long g_haptic82Ms = 0;            // millis of last 0x82 haptic OUTPUT relayed (Steam mode)
static bool          g_haptic82On = false;        // a non-zero 0x82 haptic is currently active (awaiting host stop)
static unsigned long g_hapticBlockUntil = 0;        // drop Steam haptics briefly during reconnect settle
#define RUMBLE_STUCK_MS   2500u   // Xbox: release a held rumble if no OUT packet refreshes it for this long (covers a lost stop without cutting normal short rumbles)
#define HAPTIC_QUIET_MS    300u   // Steam: after this much host silence, consider the current 0x82 haptic stream inactive
#define HAPTIC_RECONNECT_BLOCK_MS 1500u
// Seamless lizard (Steam mode): exactly like the real puck — when Steam is running it forwards the gamepad
// report 0x45; when Steam is closed it drives mouse(0x40)+keyboard(0x41) on the SAME puck interface. This is
// a PURELY USB-SIDE decision — it changes nothing about the RF poll or the host->controller relay, so the
// controller is polled identically whether Steam is up or not and can't be pushed into a non-puck state.
// Steam, while running, re-sends settings report 0x87 (lizard-off) every ~3s as a heartbeat (captured on HW).
static unsigned long g_steamAliveMs = 0;   // millis of last 0x87 settings write from Steam (its heartbeat); 0 at boot => lizard until Steam appears
#define LIZARD_WD_MS 7000u          // fall back to lizard this long after the 0x87 heartbeat stops (>2x the 3s cadence, so one missed beat won't trip it mid-session)
static bool g_autoLizard = true;    // master switch; false => Steam mode always forwards 0x45 (the prior behavior). Opening Steam also disables lizard instantly (its 0x87 heartbeat sets steamAlive).
#define BOND_FILE "/bonds.bin"
static volatile bool g_dirty = false;
static bool g_pairing = false;

static bool recEmpty(const uint8_t *r) { for (int i = 0; i < 24; i++) if (r[i]) return false; return true; }

static void genSerial() {
  uint32_t id = NRF_FICR->DEVICEID[0] ^ NRF_FICR->DEVICEID[1];
  snprintf(g_unit,  sizeof g_unit,  "FXB99602%05lX", (unsigned long)(id & 0xFFFFF));
  snprintf(g_board, sizeof g_board, "MXB99602%05lX", (unsigned long)(id & 0xFFFFF));
}
static void saveBonds() {
  InternalFS.remove(BOND_FILE);
  File f(InternalFS);
  if (f.open(BOND_FILE, FILE_O_WRITE)) {
    for (int i = 0; i < NSLOT; i++) { uint8_t u = g_slot[i].used ? 1 : 0; f.write(&u, 1); f.write(g_slot[i].rec, 24); }
    f.close();
  }
}
#define CFG_FILE "/cfg.bin"
#define CFG_MAGIC 0xC6   // bumped (added persistMode+bootMode): old cfg ignored -> clean defaults on first boot
struct Cfg { uint8_t magic, mode, mDiv, mFric, padSmooth, abSwap, back[4], pollU100, persistMode, bootMode; };
static void saveCfg(){
  Cfg c={CFG_MAGIC,g_usbMode,(uint8_t)g_mDiv,(uint8_t)g_mFric,(uint8_t)g_padSmooth,g_abSwap,
         {g_back[0],g_back[1],g_back[2],g_back[3]},(uint8_t)(g_pollUs/100),(uint8_t)(g_persistMode?1:0),g_bootMode};
  InternalFS.remove(CFG_FILE); File f(InternalFS);
  if(f.open(CFG_FILE,FILE_O_WRITE)){ f.write((uint8_t*)&c,sizeof c); f.close(); }
}
static void loadCfg(){
  Cfg c; File f(InternalFS); bool consume=false;
  if(f.open(CFG_FILE,FILE_O_READ)){
    if(f.read((uint8_t*)&c,sizeof c)==(int)sizeof c && c.magic==CFG_MAGIC){
      g_mDiv=c.mDiv?c.mDiv:64; g_mFric=c.mFric; g_padSmooth=c.padSmooth?c.padSmooth:35;
      g_abSwap=c.abSwap; for(int i=0;i<4;i++) g_back[i]=c.back[i];
      g_persistMode = c.persistMode?true:false;
      // boot-mode policy: a one-shot bootMode (set by an explicit switch when !persist) wins once and is then
      // cleared; otherwise persist->last mode, else->Steam. (poll rate stays the locked 800us default — never restored.)
      if(c.bootMode!=0xFF){ g_usbMode=(c.bootMode<=2)?c.bootMode:0; consume=true; }   // 0=Steam 1=Xbox 2=Switch (mode 3/Lizard removed; old cfgs fall back to Steam)
      else                 g_usbMode = g_persistMode ? ((c.mode<=2)?c.mode:0) : 0;
    }
    f.close();
  }
  if(consume){ g_bootMode=0xFF; saveCfg(); }   // clear the one-shot so the NEXT cold boot reverts to the default/persist policy
}
// Mode switch (chord / WebUI): persist mode if the toggle is on, else arm a one-shot so this reboot lands in
// the new mode but the next cold boot returns to Steam. Either way saveCfg + caller reboots.
static void saveMode(uint8_t m){
  if(g_persistMode){ g_usbMode=m; g_bootMode=0xFF; }
  else             { g_bootMode=m; }
  saveCfg();
}
static void loadBonds() {
  File f(InternalFS);
  if (f.open(BOND_FILE, FILE_O_READ))
    for (int i = 0; i < NSLOT; i++) { uint8_t u = 0; if (f.read(&u, 1) == 1) { g_slot[i].used = u; f.read(g_slot[i].rec, 24); } }
  f.close();
}

static bool hapticLinkUp(){
  return g_connSlot>=0 && (millis()-g_connReplyMs) < 300;
}
static bool haptic82Blocked(){
  return !hapticLinkUp() || (g_hapticBlockUntil && (int32_t)(millis()-g_hapticBlockUntil) < 0);
}
static bool haptic82PayloadOn(const uint8_t* p, uint16_t n){
  if(n<3) return false;
  for(uint16_t i=2;i<n;i++) if(p[i]) return true;   // observed form is [01 01 gain], but treat any trailing non-zero as active
  return false;
}
static void haptic82HostReport(const uint8_t* p, uint16_t n){
  if(n<3) return;
  g_haptic82Ms = millis();
  g_haptic82On = haptic82PayloadOn(p,n);
}

// ---- command channel; `slot` is the interface index (interface N == bond slot N) ----
static void handleSet(int slot, uint8_t rid, hid_report_type_t type, uint8_t const *b, uint16_t n) {
  if (type == HID_REPORT_TYPE_OUTPUT) {   // Steam haptic/LED OUTPUT reports 0x80-0x89 -> relay to controller
    if (rid >= 0x80 && rid <= 0x89 && n >= 1) {                 // wrap as a SET sub-TLV like the report-01 path
      bool haptic82 = (rid == 0x82);
      if (!haptic82 || !haptic82Blocked()) {
        uint8_t m = n > (uint16_t)(sizeof g_relayBuf - 2) ? (sizeof g_relayBuf - 2) : n;
        g_relayBuf[0] = rid; g_relayBuf[1] = m; memcpy(g_relayBuf + 2, b, m);
        g_relayN = m + 2; g_relayPend = true;
      }
      if (haptic82) haptic82HostReport(b, n);  // track on/off for the stuck-haptic watchdog
    }
    if (Serial.availableForWrite() > 80) {                      // log so we can see what Steam actually sends (e.g. glide haptics)
      Serial.printf("# OUT if%d rid=%02X n=%u:", slot, rid, n);
      for (uint16_t i = 0; i < n && i < 14; i++) Serial.printf(" %02X", b[i]);
      Serial.println();
    }
    return;
  }
  if (type != HID_REPORT_TYPE_FEATURE || n < 1) return;
  Slot &S = g_slot[slot];
  uint8_t cmd = b[0], len = (n > 1) ? b[1] : 0;
  const uint8_t *pl = b + 2; uint16_t pln = (n >= 2) ? n - 2 : 0;
  if (cmd == 0x87) g_steamAliveMs = millis();   // Steam's settings/lizard-off heartbeat (~every 3s) -> keep forwarding gamepad, suppress auto-lizard
  if (rid == 1 && n >= 2) {   // report 0x01 = raw passthrough -> queue for RF relay to the controller
    bool haptic82 = (cmd == 0x82 && len <= pln);
    if (!haptic82 || !haptic82Blocked()) {
      uint16_t m = n > sizeof g_relayBuf ? sizeof g_relayBuf : n;
      memcpy(g_relayBuf, b, m); g_relayN = m; g_relayPend = true;
    }
    if (haptic82) haptic82HostReport(pl, len);  // Steam can send haptics through feature passthrough too
  }
  if (Serial.availableForWrite() > 80) {   // log host feature writes (non-blocking)
    Serial.printf("# SET if%d rid=%02X cmd=%02X len=%u:", slot, rid, cmd, len);
    for (uint16_t i = 0; i < n && i < 14; i++) Serial.printf(" %02X", b[i]);
    Serial.println();
  }
  memset(S.resp, 0, sizeof S.resp); S.resp_len = 0;
  switch (cmd) {
    case 0x83:
      S.resp[0] = 0x83; S.resp[1] = sizeof ATTR83; memcpy(S.resp + 2, ATTR83, sizeof ATTR83); S.resp_len = 63; break;
    case 0xAE: {
      uint8_t idx = pln > 0 ? pl[0] : 1; const char *s = (idx == 0) ? g_board : (idx == 1) ? g_unit : "NA";
      S.resp[0] = 0xAE; S.resp[1] = 0x14; S.resp[2] = idx; memset(S.resp + 3, 0, 60); memcpy(S.resp + 3, s, strlen(s)); S.resp_len = 63; break; }
    case 0xB4:    // connection/version state per slot: value 0x02 = controller connected, 0x01 = not
      S.resp[0] = 0xB4; S.resp[1] = 0x01;
      S.resp[2] = (slot == g_connSlot && !g_xbox && (millis() - g_connReplyMs < 500)) ? 0x02 : 0x01;
      S.resp_len = 63; break;
    case 0xAD:
      g_pairing = (pln > 0 && pl[0] != 0); Serial.printf("# pairing %s\n", g_pairing ? "ON" : "off");
      S.resp[0] = 0xAD; S.resp[1] = 0; S.resp_len = 63; break;
    case 0xA2:                                   // write/clear THIS interface's slot
      if (len >= 24 && pln >= 24) {
        if (recEmpty(pl)) { S.used = false; memset(S.rec, 0, 24); }
        else { memcpy(S.rec, pl, 24); S.used = true; }
        g_dirty = true; Serial.printf("# slot %d %s\n", slot, recEmpty(pl) ? "cleared" : "bonded");
      }
      S.resp[0] = 0xA2; S.resp[1] = 0; S.resp_len = 63; break;
    case 0xA3:                                   // read THIS interface's slot
      S.resp[0] = 0xA3; S.resp[1] = 0x18; memset(S.resp + 2, 0, 24);
      if (S.used) memcpy(S.resp + 2, S.rec, 24); S.resp_len = 63; break;
    default:
      S.resp[0] = cmd; S.resp[1] = len; if (pln) memcpy(S.resp + 2, pl, pln > 60 ? 60 : pln); S.resp_len = 63; break;
  }
}
static uint16_t handleGet(int slot, uint8_t rid, hid_report_type_t type, uint8_t *buf, uint16_t reqlen) {
  if (type != HID_REPORT_TYPE_FEATURE) return 0;
  Slot &S = g_slot[slot];
  uint16_t n = S.resp_len ? S.resp_len : 63; if (n > reqlen) n = reqlen;
  memcpy(buf, S.resp, n); return n;
}

// one callback pair per interface (the Adafruit core routes by interface to the matching object)
#define SLOTCB(N) \
  static void setcb##N(uint8_t r, hid_report_type_t t, uint8_t const *b, uint16_t n) { handleSet(N, r, t, b, n); } \
  static uint16_t getcb##N(uint8_t r, hid_report_type_t t, uint8_t *bf, uint16_t rl) { return handleGet(N, r, t, bf, rl); }
SLOTCB(0) SLOTCB(1) SLOTCB(2) SLOTCB(3)
typedef uint16_t (*getcb_t)(uint8_t, hid_report_type_t, uint8_t *, uint16_t);
typedef void (*setcb_t)(uint8_t, hid_report_type_t, uint8_t const *, uint16_t);
static getcb_t GETCB[NSLOT] = { getcb0, getcb1, getcb2, getcb3 };
static setcb_t SETCB[NSLOT] = { setcb0, setcb1, setcb2, setcb3 };

// ===================== PHASE 2: ESB radio (firmware-derived) =====================
// Register-confirmed ESB config (RESOLVED.md): 2Mbit, 5-byte addr (BALEN=4), dynamic length
// (LFLEN=8,S1LEN=3), 16-bit preamble, big-endian, whitening off, CRC16 poly 0x11021 init 0xFFFF
// (address included). On-air = bitrev8(stored); RADIO BASE = byteswap32(bitrev8(stored)).
// Pairing rendezvous: ch2 (2402MHz), base 91 A2 A7 93 (FUN_0000b8b8 / PTR_DAT_0000ba30).
// Dongle protocol (protocol/RF_CONNECTED.md): beacon [01][seq][..][E2@5]; handshake 0xF0/0xE0/0xF4.
// Bare-metal RADIO (no SoftDevice — never start BLE); coexists with TinyUSB. Stage A: listen+log.
// Discovery/reconnect address (IBEX FUN_00019000, firmware-derived, definitive):
//   base = "ibex" (69 62 65 78), prefix = 0x10, channel = 2.
// (The 0x91A2A793 bytes in IBEX rodata precede the key string but are a *different* key,
//  NOT the discovery address — confirmed via PTR_s_ibexesb_common=0x56f99 -> "ibex...".)
static const uint8_t PAIR_BASE[4] = {0x69, 0x62, 0x65, 0x78};  // "ibex"
static uint8_t  g_rfPrefix = 0x10;     // discovery prefix (rodata byte @0x56f98)
static uint8_t  g_rfCh = 2;
static uint8_t  g_sessCh = 18;   // connected-session channel: a CLEAN data channel, off the congested BLE-adv
                                 // ch2 (=adv37). ch18 measured clean (RSSI 95/90, PER 0) on the REAL PUCK's own
                                 // channel scan and is in its active hop set {18,2,80}; ch52 (old) was mediocre.
                                 // Trackpad data bursts collide on bad channels -> reply-rate crash. Tunable 'C'.
static bool     g_rfListen = false;
static uint8_t  g_rfBase[4] = {0x69, 0x62, 0x65, 0x78};  // "ibex"
static uint8_t  rfrx[80], rftx[80];
static uint32_t g_rfRxCount = 0;

static uint8_t rfBitrev8(uint8_t x){ uint8_t y=0; for(int i=0;i<8;i++){y=(y<<1)|(x&1);x>>=1;} return y; }

// HYBRID puck-link config (firmware-derived, definitive):
//   PHY    = Ble_1Mbit (IBEX FUN_0000f108 MODE=3 is the only RADIO MODE write; ESB inherits it)
//   PCNF0  = 0x00100108  (S0LEN=1 [buf[0]=0x12 IS the S0 byte], LFLEN=8, S1INCL=1, 8-bit preamble)
//   PCNF1  = 0x02030040  (WHITEEN ON, BALEN=3 [4-byte addr], little-endian, MAXLEN=64)
//   CRC    = CRC16 poly 0x11021, init 0xFFFF, CRCCNF=2 (address INCLUDED) -- ESB override
//            (FUN_00037000); NOT the Bluetooth radio's CRC24 0x65b/CRCCNF=0x103.
//   whiten = ON, DATAWHITEIV=37 (RADIO FREQUENCY=2 = 2402MHz = BLE adv channel 37)
//   addr   = ESB addr_conv (FUN_00027a78 bitrev8 each byte): "ibex" -> BASE0=0x9646A61E.
// All tunable via CDC (M/0/2/i/w + I=crcinit) for hardware iteration.
// DECODED from real-puck capture (definitive): puck TX = Ble_2Mbit, ENDIAN=Big (MSB-first), NO whitening,
// addr "ibex", CRC16 0x11021/0xFFFF. Frame = FUN_00027e9a (static length). My Little-endian capture saw
// it bit-reversed; decode (shift3+bitrev8) recovered 0xE1 + proteus_uuid_LE + ibex_uuid_LE for both bonds.
static uint32_t g_crcinit = 0xFFFF;
static uint8_t  g_whiteiv = 37;
static uint8_t  g_mode    = RADIO_MODE_MODE_Ble_2Mbit;   // <-- was the bug (1Mbit). Real puck = 2Mbit.
static uint32_t g_pcnf0   = 0x00030008UL;                // S0LEN0, LFLEN8, S1LEN3 (ESB DPL) - CRC-VALIDATED
static uint8_t  g_statlen = 0x20;                        // (static-mode payload length; tunable 'T')
// PCNF1: ENDIAN=Big (puck transmits MSB-first), WHITEEN=0, BALEN3, MAXLEN64.
static uint32_t g_pcnf1   = 0x01040040;  // ENDIAN=Big, BALEN4, MAXLEN64 - CRC-VALIDATED
static uint32_t g_crcpoly = 0x11021UL;
static uint16_t g_crccnf  = 0x2;          // CRC16, address included
static uint8_t  g_pid     = 0;
// Connected-mode poll (Phase 2b): puck (PTX) sends 0xE7 protocol-version then 0xE3 input polls on the
// session addr/ch; controller responds in the ACK/reply with its input report (byte5==0xF1 + TLV,
// parsed by IBEX FUN_0000ba38). Tunable for on-hardware iteration (j=type, Q=len, k=enable).
static bool     g_connOn   = true;   // auto-start poll on boot (so a USB replug resumes acting as a puck)
static uint8_t  g_connType = 0xE7;   // start with protocol-version handshake, then 0xE3
static uint8_t  g_e7b = 0;           // E7 payload B-byte: 0=current(slow/awake), 1=test protocol-version-1 (ODR agent: B!=0 sets controller 'v1 fast' latch -> ~100Hz?). 'V<n>' to toggle. RISK: original RE called [00][01] "suspended" — revert to 0 if input stops.
static uint8_t  g_connLen  = 0x08;
static uint32_t g_stPoll=0, g_stF1=0, g_stF3=0; static unsigned long g_stMs=0;   // per-second poll/F1/F3 stat counters
static uint8_t  g_connF3v=0xFF;   // last protocol version the controller reported in an F3 reply (0xFF=none seen)
static uint16_t g_f1ps=0;   // last completed second's F1 rate (for the WebUSB status readout)
static uint8_t  g_lastSeq=0; static uint32_t g_stNew=0; static uint16_t g_newps=0;  // genuine new-report rate (report 0x45 seq byte changes)
static uint32_t g_stCrc=0, g_stNoRx=0;   // diagnostics: replies w/ bad CRC, and polls with no reply at all (timing vs RF-quality)
static uint32_t g_chF1[3]={0,0,0};   // diagnostic: F1 replies per poll channel {sessCh,2,80}
static uint8_t  g_chIdx=0, g_noRep=0; // (legacy hop counters; controller doesn't hop)
static uint32_t g_lastPollUs=0;   // (g_pollUs declared in the config block near the top; 'U' or WebUSB to tune)
static unsigned long g_lastSessBeacon=0;          // session keepalive beacon timer
static uint32_t g_connRx   = 0;
static uint8_t  g_balen   = 4;            // ESB 5-byte addr
// Access address (IBEX FUN_00037530 + func_0x00049e5a): BASE0 = bitrev8 each base byte, packed
// big-endian (= byteswap(bitrev8(uint32))); PREFIX0 = raw prefix (func_0x0003756c copies it). BALEN=3
// uses BASE0[31:8] + PREFIX0. For "ibex" 69 62 65 78 -> BASE0=0x9646A61E, PREFIX0=0x10.
static bool g_prefixRaw = false;   // false = bitrev8 prefix (ESB addr_conv); true = raw prefix byte
static void rfSetAddr(const uint8_t b4[4], uint8_t prefix){
  uint8_t b[4]; for(int i=0;i<4;i++) b[i]=rfBitrev8(b4[i]);
  NRF_RADIO->BASE0   = ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
  NRF_RADIO->PREFIX0 = g_prefixRaw ? prefix : rfBitrev8(prefix);
  NRF_RADIO->TXADDRESS = 0; NRF_RADIO->RXADDRESSES = 1u<<0;
}
// Promiscuous raw capture (calibration only): preamble-match on 0x55 bytes, fixed-length grab, no CRC.
// Catches any 2Mbit packet on a channel so we can read the controller's reconnect addr/prefix/framing.
#define RAWCAP 48
static bool g_rfRaw = false;
static void rfRawStart(uint8_t ch){
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->MODE=(RADIO_MODE_MODE_Nrf_2Mbit<<RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
  NRF_RADIO->MODECNF0=(RADIO_MODECNF0_RU_Fast<<RADIO_MODECNF0_RU_Pos);
#endif
  NRF_RADIO->FREQUENCY=ch; NRF_RADIO->TXPOWER=(RADIO_TXPOWER_TXPOWER_0dBm<<RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->CRCCNF=0; NRF_RADIO->PCNF0=0;
  NRF_RADIO->PCNF1=(RADIO_PCNF1_WHITEEN_Disabled<<RADIO_PCNF1_WHITEEN_Pos)
                 |(RADIO_PCNF1_ENDIAN_Big<<RADIO_PCNF1_ENDIAN_Pos)
                 |(2u<<RADIO_PCNF1_BALEN_Pos)|(RAWCAP<<RADIO_PCNF1_STATLEN_Pos)|(RAWCAP<<RADIO_PCNF1_MAXLEN_Pos);
  NRF_RADIO->BASE0=0x55555555; NRF_RADIO->PREFIX0=0x00000055;
  NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1; NRF_RADIO->PACKETPTR=(uint32_t)rfrx;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
}
// CAPTURE: listen for the CONTROLLER's transmission (test the inverted-direction hypothesis).
// RX on "ibex"/BLE_1Mbit with CRC DISABLED (any address-matched packet logged, whitened or not;
// the address is never whitened so a match still fires). Sweeps channels to find where it TXes.
static bool g_rfCap = false;
// CRC-VALIDATING capture (2Mbit, addr "ibex", CRC16 0x11021/0xFFFF). A CRC PASS pins PHY+addr+CRC+whiten.
// Data looked bit-reversed (not whitened), so sweep whiten-OFF combos first: BALEN{3,4} x CRCCNF{incl,skip}
// x ENDIAN{little,big}; then whiten-ON IV sweep as fallback. Cycle index g_capV.
struct CapCfg{ uint8_t balen; uint16_t crccnf; uint8_t endianBig; uint8_t whiten; uint8_t iv; const char*tag; };
static const CapCfg CAPC[] = {
  {3,0x2,0,0,0,"BAL3 incl LE whOFF"},   {4,0x2,0,0,0,"BAL4 incl LE whOFF"},
  {3,0x2,1,0,0,"BAL3 incl BE whOFF"},   {4,0x2,1,0,0,"BAL4 incl BE whOFF"},
  {3,0x102,0,0,0,"BAL3 skip LE whOFF"}, {4,0x102,0,0,0,"BAL4 skip LE whOFF"},
  {3,0x102,1,0,0,"BAL3 skip BE whOFF"}, {4,0x102,1,0,0,"BAL4 skip BE whOFF"},
  {3,0x2,0,1,2,"BAL3 incl LE whIV2"},   {3,0x2,0,1,37,"BAL3 incl LE whIV37"},
  {4,0x2,0,1,2,"BAL4 incl LE whIV2"},   {4,0x2,0,1,37,"BAL4 incl LE whIV37"},
};
static uint8_t g_capV=0; static uint32_t g_capPass=0;
// CRC-VALIDATING de-whiten sweep: BLE-radio framing PCNF0=0x100108 (S0LEN1/LFLEN8) + WHITEEN + CRC16
// 0x11021. Sweep DATAWHITEIV (0..127) x BALEN{3,4}. A CRC PASS de-whitens & de-frames -> clean payload + IV.
// FRAMING SWEEP: ENDIAN=Big, addr "ibex", CRC16 0x11021/0xFFFF, sweep PCNF0 x CRCCNF. A CRC PASS on the
// real puck's frame pins the exact framing (LFLEN/S1LEN/S0LEN) -> then copycat TX uses it.
static const uint32_t CAPP0[]={0x00030006,0x00030008,0x00100108,0x00000008,0x00000006,0x00130006,0x00130008,0x00100008};
// CRC configs: {CRCCNF, CRCPOLY, CRCINIT}
static const uint32_t CAPCRC[][3]={{0x2,0x11021,0xFFFF},{0x102,0x11021,0xFFFF},{0x1,0x107,0xFF},{0x101,0x107,0xFF}};
#define NCRC (sizeof CAPCRC/sizeof CAPCRC[0])
// FRAMING SWEEP v2: ENDIAN=Big, S1LEN=3 (ESB PID+noack -> the 3-bit shift), STATIC length swept,
// CRC16 0x11021 / CRC8 0x107. g_capV -> (pcnf0_idx, statlen, crc_idx).
// Firmware-correct: ESB DPL (S0L0, LFLEN6or8, S1LEN3), BALEN4 (5-byte addr), CRC8/16 addr-included.
// Sweep PCNF0{LFLEN6,8} x CRC{8,16} x BALEN{4,3} x ENDIAN{Big,Lit} = 16 combos.
static const uint32_t CAPP0B[]={0x00030006,0x00030008};  // S1LEN3 + LFLEN6 / LFLEN8 (dynamic length)
static const uint32_t CAPCRCB[][3]={{0x1,0x107,0xFF},{0x2,0x11021,0xFFFF}};
#define NCRCB 2
static void rfCapStart(uint8_t ch){
  uint8_t ci=g_capV&1;
  uint8_t balen=((g_capV>>1)&1)?3:4;            // BALEN 4 first
  uint8_t endbig=((g_capV>>2)&1)^1;             // ENDIAN Big first
  uint32_t p0=CAPP0B[(g_capV>>3)&1];
  const uint32_t* cr=CAPCRCB[ci];
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->MODE=(RADIO_MODE_MODE_Ble_2Mbit<<RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
  NRF_RADIO->MODECNF0=(RADIO_MODECNF0_RU_Fast<<RADIO_MODECNF0_RU_Pos);
#endif
  NRF_RADIO->FREQUENCY=ch;
  NRF_RADIO->PCNF0=p0;
  NRF_RADIO->PCNF1=((uint32_t)endbig<<RADIO_PCNF1_ENDIAN_Pos)|((uint32_t)balen<<RADIO_PCNF1_BALEN_Pos)
                  |(60u<<RADIO_PCNF1_MAXLEN_Pos);
  NRF_RADIO->CRCCNF=cr[0]; NRF_RADIO->CRCPOLY=cr[1]; NRF_RADIO->CRCINIT=cr[2];
  uint8_t b[4]; for(int i=0;i<4;i++) b[i]=rfBitrev8(g_rfBase[i]);
  NRF_RADIO->BASE0=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
  NRF_RADIO->PREFIX0=rfBitrev8(g_rfPrefix); NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx; memset(rfrx,0,8);
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
}
// REPLAY: capture one real-puck frame (raw, ENDIAN=Little) then re-transmit it VERBATIM (same bits,
// addr, ch) to impersonate the puck bit-for-bit -- sidesteps unknown framing/CRC. Controller must be
// bonded to the REAL puck (replayed frame carries real-puck uuids + its valid CRC).
static uint8_t g_replay[48]; static uint8_t g_replayLen=0;
static bool g_rfCapOne=false, g_rfReplay=false;
static void rfCapPoll(){
  if(!g_rfCap) return;
  if(NRF_RADIO->EVENTS_END){ NRF_RADIO->EVENTS_END=0;
    g_capPass++;
    if(g_rfCapOne){ memcpy(g_replay,rfrx,sizeof g_replay); g_replayLen=32; g_rfCapOne=false; g_rfCap=false;
      Serial.printf("# captured replay frame (%uB): ",g_replayLen);
      for(int i=0;i<g_replayLen;i++)Serial.printf("%02X",g_replay[i]); Serial.println(); return; }
    if(NRF_RADIO->CRCSTATUS&1){
      uint8_t ci=g_capV&1; uint8_t balen=((g_capV>>1)&1)?3:4; uint8_t eb=((g_capV>>2)&1)^1;
      uint32_t p0=CAPP0B[(g_capV>>3)&1];
      Serial.printf("@@@ CRCOK PCNF0=%lX BALEN=%u ENDIAN=%s POLY=%lX: ",(unsigned long)p0,balen,
        eb?"Big":"Lit",(unsigned long)CAPCRCB[ci][1]);
      for(int i=0;i<28;i++) Serial.printf("%02X",rfrx[i]); Serial.println();
    }
  }
}
static void rfReplayOnce(){
  if(!g_replayLen) return;
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->MODE=(RADIO_MODE_MODE_Ble_2Mbit<<RADIO_MODE_MODE_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
  NRF_RADIO->MODECNF0=(RADIO_MODECNF0_RU_Fast<<RADIO_MODECNF0_RU_Pos);
#endif
  NRF_RADIO->FREQUENCY=2;                                          // ch2
  NRF_RADIO->PCNF0=0;                                              // static, ENDIAN=Little -> reproduce on-air bits
  NRF_RADIO->PCNF1=(3u<<RADIO_PCNF1_BALEN_Pos)|((uint32_t)g_replayLen<<RADIO_PCNF1_STATLEN_Pos)|((uint32_t)g_replayLen<<RADIO_PCNF1_MAXLEN_Pos);
  NRF_RADIO->CRCCNF=0;                                            // CRC already inside the captured bytes
  uint8_t b[4]; for(int i=0;i<4;i++) b[i]=rfBitrev8(g_rfBase[i]);
  NRF_RADIO->BASE0=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
  NRF_RADIO->PREFIX0=rfBitrev8(g_rfPrefix); NRF_RADIO->TXADDRESS=0;
  NRF_RADIO->PACKETPTR=(uint32_t)g_replay;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_TXEN=1;
  RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
}
static void rfConfig(uint8_t ch){
  NRF_RADIO->TASKS_DISABLE = 1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->MODE = ((uint32_t)g_mode << RADIO_MODE_MODE_Pos);   // Ble_1Mbit (tunable 'M')
  NRF_RADIO->FREQUENCY = ch;
  NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
#if defined(RADIO_MODECNF0_RU_Fast)
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_RU_Fast<<RADIO_MODECNF0_RU_Pos);
#endif
  NRF_RADIO->PCNF0 = g_pcnf0;       // static length (tunable '0'); g_pcnf1=0 -> build from g_statlen/g_balen
  NRF_RADIO->PCNF1 = g_pcnf1 ? g_pcnf1 :
        ((1u<<RADIO_PCNF1_ENDIAN_Pos)              // ENDIAN=Big (puck transmits MSB-first)
        |((uint32_t)g_balen<<RADIO_PCNF1_BALEN_Pos)
        |((uint32_t)g_statlen<<RADIO_PCNF1_STATLEN_Pos)
        |((uint32_t)g_statlen<<RADIO_PCNF1_MAXLEN_Pos));   // WHITEEN=0
  NRF_RADIO->CRCCNF = g_crccnf;     // CRC16, address included
  NRF_RADIO->CRCPOLY = g_crcpoly; NRF_RADIO->CRCINIT = g_crcinit;
  NRF_RADIO->DATAWHITEIV = g_whiteiv;
  NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
}
static void rfListenStart(){
  rfConfig(g_rfCh); rfSetAddr(g_rfBase, g_rfPrefix);
  NRF_RADIO->PACKETPTR = (uint32_t)rfrx;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->EVENTS_END = 0; NRF_RADIO->TASKS_RXEN = 1;
  g_rfListen = true;
  Serial.printf("# RF listen ch%u base %02X%02X%02X%02X prefix %02X\n",
                g_rfCh, g_rfBase[0],g_rfBase[1],g_rfBase[2],g_rfBase[3], g_rfPrefix);
}
static void rfPoll(){
  if (g_rfRaw){
    if (NRF_RADIO->EVENTS_END){
      NRF_RADIO->EVENTS_END = 0;
      // filter noise: require some non-0x55/0xAA/0x00 structure
      int nz=0; for(int i=0;i<RAWCAP;i++){uint8_t b=rfrx[i]; if(b&&b!=0x55&&b!=0xAA&&b!=0xFF)nz++;}
      if (nz>=6){
        g_rfRxCount++;
        Serial.printf("RAW#%lu ch%u: ", (unsigned long)g_rfRxCount, g_rfCh);
        for (int i=0;i<RAWCAP;i++) Serial.printf("%02X", rfrx[i]);
        Serial.println();
      }
    }
    return;
  }
  if (!g_rfListen) return;
  if (NRF_RADIO->EVENTS_END){
    NRF_RADIO->EVENTS_END = 0;
    bool crcok = NRF_RADIO->CRCSTATUS & 1;
    uint8_t len = rfrx[0];
    if (len && len < 64){
      g_rfRxCount++;
      Serial.printf("RF#%lu ch%u crc%d len%u: ", (unsigned long)g_rfRxCount, g_rfCh, crcok, len);
      for (uint8_t i=0;i<=len && i<40;i++) Serial.printf("%02X ", rfrx[i]);
      Serial.println();
    }
  }
}
// Dongle is PTX: TX a beacon [01][seq][..][E2@5] then RX the controller's response (0xF0...).
// RAM layout / payload length are byte-level unknowns to pin live (S1INCL, plen tweakable).
static bool    g_rfBeacon = false;
static uint8_t g_seq = 0, g_plen = 0x18, g_s1incl = 0;
static unsigned long g_lastBeacon = 0;
static unsigned long g_lastDisc = 0;   // last discovery beacon (ch2) time
static bool    g_rfSweep = false;
static bool    g_rfHost = true;   // auto-start host beacon on boot (resumes puck role after a USB replug)
static unsigned long g_lastHop = 0;

// AUTO-SWEEP: cycle candidate radio configs while beaconing, so ONE controller search window
// (a few seconds) covers the whole space. S0LEN=1 / CRC16 0x11021 / addr "ibex" held fixed
// (firmware-confirmed); we sweep the residual unknowns: PHY (1M/2M), whitening (off/IV), CRC-addr.
// Hold MODE=Ble_1Mbit, PCNF0=0x100108 (S0LEN=1), CRC16 0x11021 cnf2; sweep the UNTESTED dims:
// BALEN (3 vs 4 = 4- vs 5-byte addr), whitening (off / iv37), prefix (bitrev vs raw), + one 2M.
struct RfCfg { uint8_t mode; uint32_t pcnf0, pcnf1; uint8_t whiteiv; uint16_t crccnf; bool prefRaw; const char* tag; };
static const RfCfg SWEEP[] = {
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02040040, 37, 0x2, false, "1M BALEN4 whiv37 pfxRev"},
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00040040, 0,  0x2, false, "1M BALEN4 whOFF pfxRev"},
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02040040, 37, 0x2, true,  "1M BALEN4 whiv37 pfxRAW"},
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00040040, 0,  0x2, true,  "1M BALEN4 whOFF pfxRAW"},
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x02030040, 37, 0x2, false, "1M BALEN3 whiv37 pfxRev"},
  {RADIO_MODE_MODE_Ble_1Mbit, 0x00100108, 0x00030040, 0,  0x2, true,  "1M BALEN3 whOFF pfxRAW"},
  {RADIO_MODE_MODE_Ble_2Mbit, 0x00100108, 0x02040040, 37, 0x2, false, "2M BALEN4 whiv37 pfxRev"},
  {RADIO_MODE_MODE_Ble_2Mbit, 0x00100108, 0x00040040, 0,  0x2, true,  "2M BALEN4 whOFF pfxRAW"},
};
static bool g_rfAuto = false; static uint8_t g_cfgIdx = 0; static unsigned long g_lastCfg = 0;
static void applyCfg(uint8_t i){
  const RfCfg &c = SWEEP[i];
  g_mode=c.mode; g_pcnf0=c.pcnf0; g_pcnf1=c.pcnf1; g_whiteiv=c.whiteiv; g_crccnf=c.crccnf; g_prefixRaw=c.prefRaw;
  Serial.printf("# cfg[%u] %s\n", i, c.tag);
}

// HOST FRAME the bonded controller waits for (IBEX FUN_00019000 verify: b[0]=0x12,b[5]=0xE1,
// b[6..10]=proteus_uuid, b[10..14]=ibex_uuid). Built like PROTEUS FUN_00027e9a. Sent on the shared
// rendezvous addr; the controller filters by the uuids in the payload, then connects.
static void rfHostFrameOnce(int slot){
  if (slot<0||slot>=NSLOT||!g_slot[slot].used) return;
  uint8_t *rec = g_slot[slot].rec;                       // [proteus_uuid 4][ibex_uuid 4][serial 16]
  // CRC-VALIDATED frame (decoded from real puck): ESB-DPL RAM = [LENGTH][S1=PID][payload(18)].
  // payload: [0]=0xE1, [1..5]=proteus_uuid LE, [5..9]=ibex_uuid LE, [9]=session channel,
  // [10..13]=0, [13..17]=session base, [17]=session prefix. Radio auto-appends CRC16 0x11021.
  memset(rftx,0,sizeof rftx);
  rftx[0]=0x12;                        // LENGTH = 18 (controller's buf[0]==0x12 check validates this)
  rftx[1]=(uint8_t)((g_pid++&3)<<1);   // S1 = PID<<1 | noack0  (matches real puck 00/02/04/06)
  rftx[2]=0xE1;                        // payload[0] marker
  memcpy(rftx+3, rec+0, 4);            // payload[1..5] proteus_uuid (LE, as bonded)
  memcpy(rftx+7, rec+4, 4);            // payload[5..9] ibex_uuid
  rftx[11]=g_sessCh;                   // payload[9] session channel: tell the controller to run the session on
                                       // the clean channel (it adopts buf[0xe]); discovery beacon still TXes on ch2
  memcpy(rftx+15, g_rfBase, 4);        // payload[13..17] session base
  rftx[19]=g_rfPrefix;                 // payload[17] session prefix
  rfConfig(g_rfCh); rfSetAddr(g_rfBase,g_rfPrefix);
  NRF_RADIO->PACKETPTR=(uint32_t)rftx;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_TXEN=1;
  RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx; rfrx[0]=0;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
  uint32_t t0=micros(); while(!NRF_RADIO->EVENTS_END && (micros()-t0)<800){}
  if (NRF_RADIO->EVENTS_END){ NRF_RADIO->EVENTS_END=0;   // ANY reception = controller answered our frame
    g_rfRxCount++; bool crcok=NRF_RADIO->CRCSTATUS&1; uint8_t len=rfrx[0];
    if (Serial.availableForWrite()>90){                  // non-blocking: don't stall the loop on CDC backpressure
      Serial.printf("*** RESP#%lu cfg[%u] ch%u crc%d rxmatch%lu len%u: ",(unsigned long)g_rfRxCount,
                    g_cfgIdx,g_rfCh,crcok,(unsigned long)NRF_RADIO->RXMATCH,len);
      for(uint8_t i=0;i<(len<40?len+2:40);i++)Serial.printf("%02X ",rfrx[i]); Serial.println(); } }
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
}
// Mid-session channel hop: advertise newCh in the host frame, transmitted on the CURRENT session channel
// (where the controller is listening), a few times so it reliably adopts it; then move our poll to newCh.
// This is the foundational test for channel-hopping/QoS — does the controller follow a live channel change?
static void rfHopTo(uint8_t newCh){
  if(g_connSlot<0 || newCh==g_sessCh) return;
  uint8_t cur=g_sessCh, savedRfCh=g_rfCh;
  g_sessCh=newCh; g_rfCh=cur;                // host frame now advertises newCh but is TXed on cur
  for(int k=0;k<6;k++){ rfHostFrameOnce(g_connSlot); delayMicroseconds(700); }
  g_rfCh=savedRfCh;                          // poll + session beacon now run on g_sessCh=newCh
}
// CONNECTED MODE (Phase 2b): copycat IS the PTX/master. It already CHOSE the session base/prefix/channel
// (advertised in the host frame the controller connected on), so no discovery/re-sync is needed. After
// connect: send E7 (proto-version 00 01), send E4 (channel map {ch,2,80}), then loop E3 input-poll (+E2
// keepalive periodically) hopping the map; RX the controller's reply each cycle (0xF1 = full input report
// w/ TLVs, 0xF3 = short status). Captured PUCK->controller bytes [LEN][S1][payload], payload[0]=type:
//   E7 00 01 (S1=01) | E4 <ch> 02 50 (S1=05) | E2 (S1=07) | E3 (S1=07).  See FUN_0000ba38 / FUN_0001b960.
// One TX->RX cycle: TX the connected packet, SHORTS straight into a short RX window to catch the reply.
// ---- XBOX-mode output: map report 0x45 -> standard gamepad (g_pad) + right-trackpad -> mouse (report 0x40) ----
// report 0x45 layout: [0]=0x45 [1]=seq [2..5]=buttons u32; analog offsets are from buttons-low-byte (rep[2]).
static int s16off(const uint8_t* r,int off){ int v=r[2+off]|(r[2+off+1]<<8); return (v&0x8000)?v-0x10000:v; }
static int u16off(const uint8_t* r,int off){ return r[2+off]|(r[2+off+1]<<8); }
static uint32_t btnsOf(const uint8_t* r){ return (uint32_t)r[2]|((uint32_t)r[3]<<8)|((uint32_t)r[4]<<16)|((uint32_t)r[5]<<24); }
// Triton button masks
#define TB_A 0x1u
#define TB_B 0x2u
#define TB_X 0x4u
#define TB_Y 0x8u
#define TB_R3 0x20u
#define TB_VIEW 0x40u
#define TB_R4 0x80u
#define TB_R5 0x100u
#define TB_RB 0x200u
#define TB_DDN 0x400u
#define TB_DRT 0x800u
#define TB_DLF 0x1000u
#define TB_DUP 0x2000u
#define TB_MENU 0x4000u
#define TB_L3 0x8000u
#define TB_STEAM 0x10000u
#define TB_L4 0x20000u
#define TB_L5 0x40000u
#define TB_LB 0x80000u
#define TB_RPADT 0x200000u
#define TB_RPADC 0x400000u
#define TB_LPADT 0x2000000u
#define TB_LPADC 0x4000000u
// Trackpad coords genuinely refresh only ~46Hz (capsense floor) but reports ship ~300Hz -> stale-repeats.
// Forwarding those raw to Steam defeats its interpolation (it sees no-motion then a jump). Smooth the pad
// X/Y in the forwarded report 0x45 (exponential LERP toward the latest genuine value each report) so Steam
// sees continuous motion. g_padSmooth = alpha% (100 = off, lower = smoother/laggier). 'D<pct>' tunes it.
// (g_padSmooth declared at top with the persisted config)
static void smoothPad(uint8_t* rep){     // rep = report 0x45; smooth LPad(@18/20) + RPad(@24/26) in place
  if(g_padSmooth>=100) return;
  float a=g_padSmooth/100.0f; uint32_t b=btnsOf(rep);
  static float lsx=0,lsy=0,rsx=0,rsy=0; static bool lpt=false,rpt=false;
  bool lt=b&TB_LPADT; int16_t lx=rep[18]|(rep[19]<<8), ly=rep[20]|(rep[21]<<8);
  if(lt){ if(!lpt){lsx=lx;lsy=ly;} else {lsx+=(lx-lsx)*a; lsy+=(ly-lsy)*a;}
    int16_t ox=(int16_t)lsx,oy=(int16_t)lsy; rep[18]=ox;rep[19]=ox>>8;rep[20]=oy;rep[21]=oy>>8; }
  lpt=lt;
  bool rt=b&TB_RPADT; int16_t rx=rep[24]|(rep[25]<<8), ry=rep[26]|(rep[27]<<8);
  if(rt){ if(!rpt){rsx=rx;rsy=ry;} else {rsx+=(rx-rsx)*a; rsy+=(ry-rsy)*a;}
    int16_t ox=(int16_t)rsx,oy=(int16_t)rsy; rep[24]=ox;rep[25]=ox>>8;rep[26]=oy;rep[27]=oy>>8; }
  rpt=rt;
}
// button code (g_back[], g_abSwap targets) -> legacy XInput bit. 0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back 10=Start 11=Guide
static uint16_t codeToXB(uint8_t c){
  switch(c){ case 1:return XB_A; case 2:return XB_B; case 3:return XB_X; case 4:return XB_Y;
    case 5:return XB_LB; case 6:return XB_RB; case 7:return XB_L3; case 8:return XB_R3;
    case 9:return XB_BACK; case 10:return XB_START; case 11:return XB_GUIDE; default:return 0; }
}
static void rfXboxGamepad(const uint8_t* r){
  uint32_t b=btnsOf(r);
  uint16_t btn=0;                                   // map Triton buttons -> XInput button word
  if(b&TB_DUP)btn|=XB_DUP;   if(b&TB_DDN)btn|=XB_DDOWN; if(b&TB_DLF)btn|=XB_DLEFT; if(b&TB_DRT)btn|=XB_DRIGHT;
  if(b&TB_MENU)btn|=XB_START; if(b&TB_VIEW)btn|=XB_BACK; if(b&TB_STEAM)btn|=XB_GUIDE;
  if(b&TB_LB)btn|=XB_LB;  if(b&TB_RB)btn|=XB_RB;
  if(b&TB_L3)btn|=XB_L3;  if(b&TB_R3)btn|=XB_R3;
  // face buttons, with optional A/B + X/Y swap (Nintendo layout)
  uint16_t fA=g_abSwap?XB_B:XB_A, fB=g_abSwap?XB_A:XB_B, fX=g_abSwap?XB_Y:XB_X, fY=g_abSwap?XB_X:XB_Y;
  if(b&TB_A)btn|=fA; if(b&TB_B)btn|=fB; if(b&TB_X)btn|=fX; if(b&TB_Y)btn|=fY;
  // back paddles -> configurable mapping (default L4->LB, R4->RB, L5->L3, R5->R3)
  if(b&TB_L4)btn|=codeToXB(g_back[0]); if(b&TB_R4)btn|=codeToXB(g_back[1]);
  if(b&TB_L5)btn|=codeToXB(g_back[2]); if(b&TB_R5)btn|=codeToXB(g_back[3]);
  uint8_t lt=u16off(r,4)>>8, rt=u16off(r,6)>>8;     // triggers u16 -> u8
  xinputSend(btn, lt, rt, (int16_t)s16off(r,8), (int16_t)s16off(r,10),   // L stick X/Y
                          (int16_t)s16off(r,12), (int16_t)s16off(r,14)); // R stick X/Y
}
// Right pad -> mouse, riding ALONGSIDE the XInput gamepad on a second HID-mouse interface. Same glide model
// as Lizard's right pad (velocity injected from pad deltas, friction decay, sub-pixel carry). This is purely
// a USB-side translation of the received 0x45 report — it touches NOTHING about the RF poll or relay, so the
// controller behaves exactly as it does for the bare gamepad. RPad click = left button, LPad click = right.
// Xbox drops CDC/WebUSB, so the only active USB interfaces are XInput (MI_00) + boot mouse (MI_01).
static void rfXboxMouse(const uint8_t* r){
  uint32_t b=btnsOf(r);
  static int prx=0,pry=0; static bool prt=false; static float vx=0,vy=0,rmx=0,rmy=0; static uint8_t pmb=0;
  bool rtouch=b&TB_RPADT; int rx=s16off(r,22), ry=s16off(r,24);
  if(rtouch){ if(prt){ vx+=(rx-prx); vy+=(ry-pry); } prx=rx; pry=ry; } prt=rtouch;
  float mxf=vx/(float)(g_mDiv*10)+rmx, myf=-(vy/(float)(g_mDiv*10))+rmy;   // Y inverted for screen coords
  int dx=(int)mxf, dy=(int)myf; rmx=mxf-dx; rmy=myf-dy;                    // sub-pixel carry
  if(dx>127)dx=127; if(dx<-127)dx=-127; if(dy>127)dy=127; if(dy<-127)dy=-127;
  float f=g_mFric/100.0f; vx*=f; vy*=f; if(vx>-1&&vx<1)vx=0; if(vy>-1&&vy<1)vy=0;   // friction = glide/decay
  uint8_t mb=((b&TB_RPADC)?1:0)|((b&TB_LPADC)?2:0);  // RPad click = left, LPad click = right
  if(dx||dy||mb!=pmb){ pmb=mb;
    hid_mouse_report_t m; m.buttons=mb; m.x=(int8_t)dx; m.y=(int8_t)dy; m.wheel=0; m.pan=0;
    if(g_mouse.ready()) g_mouse.sendReport(0,&m,sizeof m);
  }
}
// ===================== Seamless LIZARD (Steam mode, when Steam is closed) — desktop keyboard + mouse =====
// The controller's native "lizard" desktop behavior, reproduced on the copycat side so the device is a
// driverless keyboard+mouse when Steam isn't running. Driven from the Steam path (report 0x45 -> mouse 0x40 +
// keyboard 0x41 on the puck interface) and parameterized by the target HID device + report IDs. Canonical
// Valve Steam Controller firmware map (SC1, the documented reference; SC2 presumed-similar): right pad ->
// mouse (trackball glide), left pad -> scroll wheel + middle-click on press, R-trigger / R-pad-click -> left
// mouse, L-trigger -> right mouse; A=Enter B=Esc X=PgUp Y=PgDn, D-pad & left-stick -> arrow keys, LB=LeftCtrl
// RB=LeftAlt, Back/View=Tab, Start/Menu=Esc, Steam=none. The mouse reuses the SAME velocity+friction+sub-pixel
// GLIDE model as Xbox mode (g_mDiv = sensitivity divisor, g_mFric = friction% — both live-tunable).
static void rfLizard(const uint8_t* r, Adafruit_USBD_HID* mdev, Adafruit_USBD_HID* kdev, uint8_t mrid, uint8_t krid){
  uint32_t b=btnsOf(r);
  // --- right pad -> mouse motion with glide (mirrors rfXboxMouse) ---
  static int prx=0,pry=0; static bool prt=false; static float vx=0,vy=0,rmx=0,rmy=0;
  bool rtouch=b&TB_RPADT; int rx=s16off(r,22), ry=s16off(r,24);
  if(rtouch){ if(prt){ vx+=(rx-prx); vy+=(ry-pry); } prx=rx; pry=ry; } prt=rtouch;
  float mxf=vx/(float)(g_mDiv*10)+rmx, myf=-(vy/(float)(g_mDiv*10))+rmy;   // Y inverted; *10 = desktop-cursor sensitivity (g_mDiv 64 -> eff 640). Lower g_mDiv via WebUI slider = faster.
  int dx=(int)mxf, dy=(int)myf; rmx=mxf-dx; rmy=myf-dy;          // sub-pixel carry
  if(dx>127)dx=127; if(dx<-127)dx=-127; if(dy>127)dy=127; if(dy<-127)dy=-127;
  float f=g_mFric/100.0f; vx*=f; vy*=f; if(vx>-1&&vx<1)vx=0; if(vy>-1&&vy<1)vy=0;   // friction = glide/decay
  // --- left pad -> vertical scroll wheel (no momentum; only while touching, coarse) ---
  static int ply=0; static bool plt=false; static float sacc=0;
  bool ltouch=b&TB_LPADT; int ly=s16off(r,18);
  if(ltouch){ if(plt){ sacc += (ly-ply)/(float)(g_mDiv*24); } ply=ly; } else sacc=0; plt=ltouch;
  int dw=(int)sacc; sacc-=dw; if(dw>15)dw=15; if(dw<-15)dw=-15;   // finger up = wheel up (positive)
  // --- mouse buttons: left=R-pad-click|R-trigger, right=L-trigger, middle=L-pad-click ---
  uint8_t rtrig=u16off(r,6)>>8, ltrig=u16off(r,4)>>8;
  uint8_t mbtn=0;
  if((b&TB_RPADC)||rtrig>180) mbtn|=1;
  if(ltrig>180)               mbtn|=2;
  if(b&TB_LPADC)              mbtn|=4;
  static uint8_t pmbtn=0;
  if(dx||dy||dw||mbtn!=pmbtn){ pmbtn=mbtn;
    hid_mouse_report_t m; m.buttons=mbtn; m.x=(int8_t)dx; m.y=(int8_t)dy; m.wheel=(int8_t)dw; m.pan=0;
    if(mdev->ready()) mdev->sendReport(mrid,&m,sizeof m);
  }
  // --- keyboard: modifiers + up to 6 keycodes ---
  uint8_t mod=0, kc[6]={0,0,0,0,0,0}, nk=0;
  if(b&TB_LB) mod|=KEYBOARD_MODIFIER_LEFTCTRL;
  if(b&TB_RB) mod|=KEYBOARD_MODIFIER_LEFTALT;
  #define LZK(cond,code) do{ if((cond)&&nk<6) kc[nk++]=(code); }while(0)
  LZK(b&TB_A,    HID_KEY_ENTER);
  LZK(b&TB_B,    HID_KEY_ESCAPE);
  LZK(b&TB_X,    HID_KEY_PAGE_UP);
  LZK(b&TB_Y,    HID_KEY_PAGE_DOWN);
  LZK(b&TB_VIEW, HID_KEY_TAB);
  LZK(b&TB_MENU, HID_KEY_ESCAPE);
  int sx=s16off(r,8), sy=s16off(r,10);   // left stick (XInput sign: +Y = up); deflect ~37% acts as a d-pad
  LZK((b&TB_DUP)||sy> 12000, HID_KEY_ARROW_UP);
  LZK((b&TB_DDN)||sy<-12000, HID_KEY_ARROW_DOWN);
  LZK((b&TB_DLF)||sx<-12000, HID_KEY_ARROW_LEFT);
  LZK((b&TB_DRT)||sx> 12000, HID_KEY_ARROW_RIGHT);
  #undef LZK
  static uint8_t pmod=0, pkc[6]={0,0,0,0,0,0};
  bool chg=(mod!=pmod); for(int i=0;i<6;i++) if(kc[i]!=pkc[i]) chg=true;
  if(chg){ pmod=mod; for(int i=0;i<6;i++) pkc[i]=kc[i];
    uint8_t krep[8]={mod,0,kc[0],kc[1],kc[2],kc[3],kc[4],kc[5]};
    if(kdev->ready()) kdev->sendReport(krid,krep,8);
  }
}
// ===================== SWITCH mode (mode 2) — HORIPAD / Pokkén Tournament Pro Pad =====================
// Targets a REAL Switch console: a plain HID gamepad (HORI 0F0D:0092) the console accepts on plug-in with
// NO handshake / SPI calibration / encryption — unlike the genuine Pro Controller (057E:2009), whose USB
// init the console handles fussily. This is the descriptor every DIY Switch controller copies (the LUFA
// "Splatoon printer" descriptor). Also recognized by SDL/Steam on PC. 8-byte input report streamed at
// ~125Hz: [btn_lo][btn_hi][hat][LX][LY][RX][RY][vendor], sticks uint8 center 0x80.
static const uint8_t SWITCH_HID_DESC[]={
  0x05,0x01, 0x09,0x05, 0xA1,0x01,           // Usage Page (Generic Desktop), Usage (Game Pad), Collection (App)
  0x15,0x00, 0x25,0x01, 0x35,0x00, 0x45,0x01, 0x75,0x01, 0x95,0x10,   // 16 x 1-bit buttons
  0x05,0x09, 0x19,0x01, 0x29,0x10, 0x81,0x02,
  0x05,0x01, 0x25,0x07, 0x46,0x3B,0x01, 0x75,0x04, 0x95,0x01, 0x65,0x14, 0x09,0x39, 0x81,0x42,  // hat switch (4 bits)
  0x65,0x00, 0x95,0x01, 0x81,0x01,           // 4-bit padding to a byte
  0x26,0xFF,0x00, 0x46,0xFF,0x00, 0x09,0x30, 0x09,0x31, 0x09,0x32, 0x09,0x35, 0x75,0x08, 0x95,0x04, 0x81,0x02, // X,Y,Z,Rz
  0x06,0x00,0xFF, 0x09,0x20, 0x95,0x01, 0x81,0x02,           // vendor byte (input)
  0x0A,0x21,0x26, 0x95,0x08, 0x91,0x02,                      // vendor (output, 8 bytes; unused)
  0xC0
};
static uint32_t g_swBtns=0; static int16_t g_swLX=0,g_swLY=0,g_swRX=0,g_swRY=0;  // latest decoded RF frame
static unsigned long g_swLastMs=0;
Adafruit_USBD_HID g_switch;   // HORIPAD gamepad HID interface (mode 2)
// HORIPAD/Switch button bits: Y=1 B=2 A=4 X=8 L=10 R=20 ZL=40 ZR=80 Minus=100 Plus=200 LClick=400 RClick=800 Home=1000 Capture=2000
static inline uint8_t swStick(int16_t v,bool invert){   // int16 (center 0) -> uint8 (center 0x80)
  int32_t a = 0x80 + (invert ? -((int32_t)v>>8) : ((int32_t)v>>8));
  if(a<0)a=0; if(a>255)a=255; return (uint8_t)a;
}
// back-paddle code (g_back[]) -> Switch button bit. 0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back(Minus) 10=Start(Plus) 11=Guide(Home)
static uint16_t codeToSwitch(uint8_t c, uint16_t fA,uint16_t fB,uint16_t fX,uint16_t fY){
  switch(c){ case 1:return fA; case 2:return fB; case 3:return fX; case 4:return fY;
    case 5:return 0x10; case 6:return 0x20; case 7:return 0x400; case 8:return 0x800;
    case 9:return 0x100; case 10:return 0x200; case 11:return 0x1000; default:return 0; }
}
static void switchBuildHoripad(uint8_t out[8]){
  uint32_t b=g_swBtns; uint16_t btn=0;
  // face buttons with optional A/B + X/Y swap (Nintendo physical-vs-label layout)
  uint16_t fY = g_abSwap?0x08:0x01, fB = g_abSwap?0x04:0x02, fA = g_abSwap?0x02:0x04, fX = g_abSwap?0x01:0x08;
  if(b&TB_Y)btn|=fY; if(b&TB_B)btn|=fB; if(b&TB_A)btn|=fA; if(b&TB_X)btn|=fX;
  if(b&TB_LB)btn|=0x10; if(b&TB_RB)btn|=0x20;                 // L, R
  if(b&0x8000000u)btn|=0x40; if(b&0x800000u)btn|=0x80;        // ZL(LTrigClick), ZR(RTrigClick)
  if(b&TB_VIEW)btn|=0x100; if(b&TB_MENU)btn|=0x200;           // Minus, Plus
  if(b&TB_L3)btn|=0x400; if(b&TB_R3)btn|=0x800;               // LClick, RClick
  if(b&TB_STEAM)btn|=0x1000;                                  // Home
  // back paddles -> configurable mapping (same g_back[] as Xbox: default L4->LB R4->RB L5->L3 R5->R3)
  if(b&TB_L4)btn|=codeToSwitch(g_back[0],fA,fB,fX,fY); if(b&TB_R4)btn|=codeToSwitch(g_back[1],fA,fB,fX,fY);
  if(b&TB_L5)btn|=codeToSwitch(g_back[2],fA,fB,fX,fY); if(b&TB_R5)btn|=codeToSwitch(g_back[3],fA,fB,fX,fY);
  bool u=b&TB_DUP,d=b&TB_DDN,l=b&TB_DLF,r=b&TB_DRT;           // hat: 0=N..7=NW, 8=neutral
  uint8_t hat=8;
  if(u&&r)hat=1; else if(r&&d)hat=3; else if(d&&l)hat=5; else if(l&&u)hat=7;
  else if(u)hat=0; else if(r)hat=2; else if(d)hat=4; else if(l)hat=6;
  out[0]=btn&0xFF; out[1]=btn>>8; out[2]=hat;
  out[3]=swStick(g_swLX,false); out[4]=swStick(g_swLY,true);  // HID Y is down-positive -> invert
  out[5]=swStick(g_swRX,false); out[6]=swStick(g_swRY,true);
  out[7]=0;
}
static void swStream(){   // call from loop(); stream the 8-byte HORIPAD report at ~125Hz (no handshake needed)
  if(g_usbMode!=2 || !g_switch.ready()) return;
  if(millis()-g_swLastMs < 8) return; g_swLastMs=millis();
  uint8_t p[8]; switchBuildHoripad(p);
  g_switch.sendReport(0, p, sizeof p);   // report-id-less descriptor
}

// RECIPE (IBEX disasm @0x18d80): controller streams 0xF1 input report ONLY when (a) connstate==3 [host
// frame E1 establishes it], (b) we mark host-awake via 0xE7 payload [00][00] (vs [00][01]=suspended,
// handler @0x193aa), and (c) EACH 0xE3 poll carries a GET-report-0x45 sub-TLV [len=02][subtype=01][id=45]
// [param] (parser @0x1948c; F1 builder @0x20ca4 arms a one-shot latch @0x2000d36e cleared after each F1 ->
// no free-run stream, 1 F1 per GET). F1 reply = type 0xF1 @rfrx[2] then TLV type4(analog/buttons)+
// type6(per-module gyro/accel/sticks).
static uint8_t  g_connSt    = 0;         // 0=announce awake, 1=poll loop
static uint8_t  g_connStep  = 0;         // repeat counter within a state
static uint16_t g_connPoll  = 0;         // poll counter (re-assert awake every 64th)
static uint32_t g_connF1    = 0;         // count of 0xF1 input reports seen
static uint8_t  g_getParam  = 0x00;      // GET report 0x45 param byte (try 0x00, fall back 0x2D) - 'q' cmd
static uint8_t  g_e3mode = 1, g_e3pid = 0;  // E3-poll PID/S1 mode: 0=fixed 0x07, 1=cycle PID(noack1), 2=cycle PID(noack0). DEFAULT 1: cycling the ESB PID drains the controller's report queue (~400 new/s vs ~60 with a fixed PID) — THE rate fix. 'e<n>' to A/B.
static bool     g_connVerbose = false;   // true=full multi-line debug dump; false=compact "I45 <hex>" stream
static unsigned long g_lastStream = 0;   // throttle for compact UI stream
static unsigned long g_connCooldown = 0; // set on 0xF2 disconnect; pauses beacon+poll so a powering-off controller can sleep
// (g_connReplyMs, g_connSlot declared near g_slot so the USB feature handler can read them)
// TX one connected packet [LEN][S1][payload] on channel ch, then RX the reply into rfrx. Decodes 0xF1.
static uint8_t rfConnTx(uint8_t ch, uint8_t s1, const uint8_t* payload, uint8_t plen){
  memset(rftx,0,sizeof rftx);
  rftx[0]=plen;                          // LENGTH = payload byte count
  rftx[1]=s1;                            // S1 (type-specific)
  memcpy(rftx+2, payload, plen);         // payload[0]=type byte, then data/TLVs
  rfConfig(ch); rfSetAddr(g_rfBase,g_rfPrefix);
  NRF_RADIO->PACKETPTR=(uint32_t)rftx;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_TXEN=1;
  RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx; rfrx[0]=0;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk;       // RXEN->READY->START; catch the reply
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
  g_stPoll++;
  uint32_t t0=micros(); while(!NRF_RADIO->EVENTS_END && (micros()-t0)<g_rxWin){}  // RX window (tunable 'r'): caps poll rate ~1e6/rxWin
  uint8_t rxlen=0;
  if (NRF_RADIO->EVENTS_END){ NRF_RADIO->EVENTS_END=0;
    bool crcok=NRF_RADIO->CRCSTATUS&1; rxlen=rfrx[0];
    if(!crcok){ g_stCrc++; g_qosBad++; }                // reply arrived but CRC failed -> RF quality (channel/interference)
    if (crcok && rxlen && rxlen<=64){                   // F1 report ~46B, so allow up to MAXLEN
      g_connRx++; g_connReplyMs=millis();               // link alive -> loop() suppresses the redundant E1 beacon
      if(rfrx[2]==0xF1) g_stF1++;
      uint8_t rtype=rfrx[2];                            // reply type byte (proven offset from captures)
      if(rtype==0xF2) g_connCooldown=millis();          // controller disconnecting/powering off -> back off 2.5s
      if(rtype==0xF3){                                  // F3 = controller status/version reply (ODR agent: reply to E7 version handshake, byte[6]=version)
        g_stF3++; g_connF3v=rfrx[6];
        if(g_connVerbose && Serial.availableForWrite()>40){ Serial.print("  F3 "); for(uint8_t i=0;i<(rxlen+2<32?rxlen+2:32);i++)Serial.printf("%02X",rfrx[i]); Serial.println(); }
      }
      bool isF1=(rtype==0xF1);
      if(isF1){
        g_connF1++;
        int idx=3, end=rxlen+2;                         // walk ALL type6 TLVs (= HID report 0x45). idx is INT, not
        const uint8_t* lastRep=nullptr; uint8_t lastTlen=0;  // uint8_t: a tlen of 0xFE would make idx+=tlen+2 wrap
        while(idx+1<end){                               // mod-256 back to itself -> infinite loop -> USB hang/"crash".
          uint8_t tlen=rfrx[idx], ttype=rfrx[idx+1];    // pace with the real puck — taking only [0] halved our rate.
          if(tlen==0) break;
          if(ttype==6 && tlen>=2 && rfrx[idx+2]==0x45){
            const uint8_t* rep=&rfrx[idx+2];            // report 0x45: [0x45][seq][buttons u32]...
            bool fresh=(rep[1]!=g_lastSeq); if(fresh){ g_stNew++; g_lastSeq=rep[1]; }  // genuine new report vs stale poll-repeat
            uint32_t bb=btnsOf(rep);
            // cache the latest decoded frame for the Switch streamer + Xbox/Steam paths
            g_swBtns=bb; g_swLX=(int16_t)s16off(rep,8); g_swLY=(int16_t)s16off(rep,10);
            g_swRX=(int16_t)s16off(rep,12); g_swRY=(int16_t)s16off(rep,14);
            if(g_usbMode==2){                           // SWITCH: streamed from loop() (swStream); nothing to send here
            } else if(g_xbox){                          // XBOX: standard gamepad + right-pad mouse (2nd interface)
              rfXboxGamepad(rep); rfXboxMouse(rep);
            } else if(g_connSlot>=0 && g_connSlot<NSLOT){  // STEAM (puck interface)
              // Seamless lizard, exactly like the real puck: while Steam's 0x87 heartbeat is alive -> forward
              // the gamepad report 0x45; once it stops (Steam closed) -> drive mouse(0x40)+keyboard(0x41) on the
              // SAME puck interface (the cloned descriptor exposes both). Pure USB-side switch — RF/relay unchanged.
              bool steamAlive = g_steamAliveMs && (millis()-g_steamAliveMs < LIZARD_WD_MS);
              if(g_autoLizard && !steamAlive){
                rfLizard(rep, &hid[g_connSlot], &hid[g_connSlot], 0x40, 0x41);
              } else {
                uint8_t blen=tlen-1; if(blen>45)blen=45;  // body after the 0x45 id byte
                smoothPad((uint8_t*)rep);                 // interpolate the 46Hz pad coords -> continuous motion for Steam
                // forward only FRESH reports (the real puck dedupes -> Steam gets a clean unique stream; sending
                // stale repeats makes Steam's velocity/smoothing stair-step). g_fwdNewOnly toggles for A/B.
                if((fresh || !g_fwdNewOnly) && g_slot[g_connSlot].used && hid[g_connSlot].ready())
                  hid[g_connSlot].sendReport(0x45, rep+1, blen);   // Steam/SDL: input report -> "connected"
              }
            }
            lastRep=rep; lastTlen=tlen;
          }
          idx+=tlen+2;
        }
        // mode-switch chord (back4 R4|L4|R5|L5 + A/X/Y), checked ONCE per frame + DEBOUNCED: a single
        // garbled/misaligned report must not trigger saveMode+reset (that looked like a crash under heavy input).
        { static uint8_t chWant=0xFF, chCnt=0;
          const uint32_t BACK4=TB_R4|TB_L4|TB_R5|TB_L5; uint8_t want=0xFF;
          if((g_swBtns&BACK4)==BACK4){ if(g_swBtns&TB_A)want=0; else if(g_swBtns&TB_X)want=1; else if(g_swBtns&TB_Y)want=2; }  // back4 + A=Steam X=Xbox Y=Switch (lizard is automatic in Steam mode now)
          if(want!=0xFF && want==chWant){ if(++chCnt>=12 && want!=g_usbMode){ saveMode(want); delay(40); NVIC_SystemReset(); } }
          else { chWant=want; chCnt=(want!=0xFF)?1:0; }
        }
        // compact stream for rf_controller_ui.py — NON-BLOCKING: skip if CDC TX is backed up (a blocking
        // Serial.print stalls the RF+USB loop -> jaggy input). One line/frame using the last record.
        if(lastRep && !g_connVerbose && Serial.availableForWrite()>110 && millis()-g_lastStream>=4){
          g_lastStream=millis(); Serial.print("I45 ");
          for(uint8_t i=0;i<lastTlen;i++)Serial.printf("%02X",lastRep[i]); Serial.println();
        }
      }
      if(g_connVerbose){
        Serial.printf("%s CRX#%lu txtype%02X ch%u len%u: ",isF1?"<<<F1":(rtype==0xF3?"  F3":"  rx"),
                      (unsigned long)g_connRx,payload[0],ch,rxlen);
        for(uint8_t i=0;i<(rxlen+2<=66?rxlen+2:66);i++)Serial.printf("%02X",rfrx[i]); Serial.println();
      }
    } else rxlen=0;
  } else { g_stNoRx++; g_qosBad++; }                   // RX window expired with no packet at all -> turnaround/timing or controller silent
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  return rxlen;
}
// Drive the connected-mode sequence one step per call (from loop() while g_connOn). Camps on g_rfCh (the
// host-frame channel the controller connected on); the E1 beacon (cmd 'H') runs in parallel as keepalive.
static void rfConnStep(){
  g_connSlot=-1; for(int s=0;s<NSLOT;s++) if(g_slot[s].used){g_connSlot=s;break;}
  if(g_connSlot<0) return;               // need a bonded slot (session established by host frame)
  if(g_connSt==0){                       // announce HOST AWAKE: E7 00 00, a few times
    uint8_t p[3]={0xE7,0x00,g_e7b}; rfConnTx(g_sessCh,0x01,p,3);
    if(++g_connStep>=4){ g_connSt=1; g_connStep=0; Serial.println("# CONN: awake announced -> polling GET report 0x45"); }
  } else {                               // poll loop: E3 + GET report 0x45 every poll; re-assert awake periodically
    if((uint32_t)(micros()-g_lastPollUs) < g_pollUs) return;   // CONTROLLED CADENCE: poll ~every g_pollUs,
    g_lastPollUs=micros();                                     // not as-fast-as-possible (over-polling starves replies)
    uint8_t ch=g_sessCh;                                       // controller stays on the session channel (no hop)
    if((g_connPoll & 0x1F)==0){ uint8_t pa[3]={0xE7,0x00,g_e7b}; rfConnTx(ch,0x01,pa,3); }   // re-assert awake/version
    if(!g_relayPend && g_testHaptic){                // buzz-hunt: synthesize a haptic relay (output 0x82 [01 01 F7])
      g_relayBuf[0]=0x82; g_relayBuf[1]=3; g_relayBuf[2]=0x01; g_relayBuf[3]=0x01; g_relayBuf[4]=0xF7; g_relayPend=true; g_testHaptic--;
    }
    if(g_relayPend){                                 // relay host cmd: [op][len][sub][rid][payload] (op/sub tunable for the buzz hunt)
      uint8_t rid=g_relayBuf[0], rl=g_relayBuf[1]; if(rl>18)rl=18;
      uint8_t p[24]; p[0]=g_relayOp; p[1]=(uint8_t)(1+rl); p[2]=g_relaySub; p[3]=rid;   // len = sub-id + payload bytes
      for(uint8_t i=0;i<rl;i++) p[4+i]=g_relayBuf[2+i];
      rfConnTx(ch,0x07,p,(uint8_t)(4+rl));
      g_relayPend=false;
    } else {
      uint8_t p[5]={0xE3,0x02,0x01,0x45,g_getParam}; // E3 + TLV [len=02][subtype=01 GET][id=0x45][param]
      uint8_t s1 = (g_e3mode==1) ? (uint8_t)((((g_e3pid++)&3)<<1)|1)   // cycle PID (S1 1,3,5,7), NO_ACK=1
                 : (g_e3mode==2) ? (uint8_t)(((g_e3pid++)&3)<<1)       // cycle PID (S1 0,2,4,6), NO_ACK=0
                 : 0x07;                                               // fixed (current default; matches captured puck poll)
      uint8_t rx=rfConnTx(ch,s1,p,5);
      if(rx) g_chF1[0]++;
    }
    g_connPoll++;
  }
}
// SESSION SNIFFER (Phase 2b capture): phase 0 RX the real puck's host frame on "ibex"/ch2 (CRC16
// config) to learn the random session base/prefix/channel; phase 1 RX that session address on the
// hop channels {hostch,2,80} to capture the live connected exchange (0xE3 poll + 0xF1 input reply).
static bool g_sniff=false; static uint8_t g_sniffPh=0; static uint32_t g_sniffN=0;
static uint8_t g_sbase[4]={0,0,0,0}, g_sprefix=0; static uint8_t g_schan[3]={78,2,80}; static uint8_t g_schi=0;
static unsigned long g_sniffHop=0;
static uint8_t g_sniffPark=0;   // phase-1: 0=park on learned primary g_schan[0]; else park on this literal channel ('g<n>')
static void rfSniffStart(){
  uint8_t ch = (g_sniffPh==0)?2:(g_sniffPark?g_sniffPark:g_schan[0]);   // PARK (no hop): primary carries the traffic
  const uint8_t* base = (g_sniffPh==0)?g_rfBase:g_sbase;
  uint8_t pfx = (g_sniffPh==0)?g_rfPrefix:g_sprefix;
  rfConfig(ch);                         // Ble_2Mbit, PCNF0=0x30008, ENDIAN big, BALEN4, CRC16 0x11021
  uint8_t b[4]; for(int i=0;i<4;i++) b[i]=rfBitrev8(base[i]);
  NRF_RADIO->BASE0=((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
  NRF_RADIO->PREFIX0=rfBitrev8(pfx); NRF_RADIO->TXADDRESS=0; NRF_RADIO->RXADDRESSES=1;
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx; memset(rfrx,0,4);
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
}
static void rfSniffPoll(){
  if(!g_sniff) return;
  if(NRF_RADIO->EVENTS_END){ NRF_RADIO->EVENTS_END=0;
    bool crcok=NRF_RADIO->CRCSTATUS&1; uint8_t len=rfrx[0];
    if(crcok && len && len<40){
      g_sniffN++;
      if(g_sniffPh==0 && rfrx[2]==0xE1){           // host frame -> learn session params + DUMP FULL (app-data may ride here)
        memcpy(g_sbase,rfrx+15,4); g_sprefix=rfrx[19]; g_schan[0]=rfrx[11];
        Serial.printf("# HOSTFRAME ch=%u sbase=%02X%02X%02X%02X prefix=%02X len%u FULL: ",
          rfrx[11],g_sbase[0],g_sbase[1],g_sbase[2],g_sbase[3],g_sprefix,len);
        { uint8_t dn=(len+2<64)?len+2:64; for(uint8_t i=0;i<dn;i++)Serial.printf("%02X",rfrx[i]); } Serial.println();
        g_sniffPh=1; g_schi=0; rfSniffStart();
      } else if(g_sniffPh==1){
        uint8_t ty=rfrx[2];                          // 0xF1=input, 0x12/0xE1/0xE4=puck host-frame w/ app-data
        const char* tag = (ty==0xF1)?"<<<INPUT":(ty==0x12||ty==0xE1||ty==0xE4)?">>>HOSTFR":"  pkt";
        Serial.printf("%s SNIFF#%lu ch%u len%u: ",tag,(unsigned long)g_sniffN,g_schan[g_schi],len);
        uint8_t dn=(len+2<64)?len+2:64;              // FULL frame (app-data TLVs sit near the end)
        for(uint8_t i=0;i<dn;i++)Serial.printf("%02X",rfrx[i]); Serial.println();
      }
    }
  }
}
// SCAN-THEN-RESPOND (dongle role): listen on ch2/91A2A793 for the controller's advertisement,
// and the instant one arrives, TX the host frame (its ack). Matches IBEX 'scan-before-initiating'.
static uint8_t g_hostTx[48]; static int g_hostLen=0; static bool g_rfRespond=false; static uint32_t g_advCount=0;
static void buildHostTx(){
  int slot=-1; for(int i=0;i<NSLOT;i++) if(g_slot[i].used){slot=i;break;}
  memset(g_hostTx,0,sizeof g_hostTx); g_hostLen=0; if(slot<0) return;
  // S0LEN=1 RAM layout: [0]=S0=0x12, [1]=LENGTH, raw RAM offsets validated by controller.
  memset(g_hostTx,0,sizeof g_hostTx);
  g_hostTx[0]=0x12; g_hostTx[1]=0x20; g_hostTx[3]=1; g_hostTx[5]=0xE1;
  memcpy(g_hostTx+6,g_slot[slot].rec,4); memcpy(g_hostTx+10,g_slot[slot].rec+4,4);
  g_hostTx[0xe]=2; memcpy(g_hostTx+0x12,g_rfBase,4); g_hostTx[0x16]=g_rfPrefix;
  g_hostLen=0x22;
}
static void rfRespondStart(){
  buildHostTx();
  rfConfig(2); rfSetAddr(g_rfBase,g_rfPrefix);            // RX ch2 / 91A2A793
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1; g_rfRespond=true;
  Serial.printf("# RESPOND mode: RX ch2/91A2A793, reply host frame (len%d)\n", g_hostLen);
}
static void rfRespondPoll(){
  if(!g_rfRespond) return;
  if(NRF_RADIO->EVENTS_END){
    NRF_RADIO->EVENTS_END=0; uint8_t len=rfrx[0];
    if(len && len<64){
      g_advCount++;
      // reply: TX the host frame immediately (ack)
      NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
      NRF_RADIO->PACKETPTR=(uint32_t)g_hostTx;
      NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
      NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_TXEN=1;
      RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
      Serial.printf("ADV#%lu len%u: ",(unsigned long)g_advCount,len);
      for(uint8_t i=0;i<=len&&i<32;i++)Serial.printf("%02X",rfrx[i]); Serial.println();
      // re-arm RX
      NRF_RADIO->PACKETPTR=(uint32_t)rfrx;
      NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk;
      NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
    }
  }
}
static void rfBeaconOnce(){
  uint8_t pl[48]; memset(pl,0,sizeof pl);
  pl[0]=0x01; pl[1]=g_seq++; pl[5]=0xE2;                 // dongle beacon (FUN_00027ed8 format)
  rfConfig(g_rfCh); rfSetAddr(g_rfBase,g_rfPrefix);
  if (g_s1incl) NRF_RADIO->PCNF0 |= (1u<<RADIO_PCNF0_S1INCL_Pos);
  // RAM: [LENGTH][ (S1 if S1INCL) ][payload]
  rftx[0]=0; rftx[1]=g_plen; memcpy(rftx+2,pl,g_plen);
  NRF_RADIO->PACKETPTR=(uint32_t)rftx;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->EVENTS_DISABLED=0; NRF_RADIO->TASKS_TXEN=1;
  RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
  // RX window for the controller's response
  NRF_RADIO->PACKETPTR=(uint32_t)rfrx; rfrx[0]=0;
  NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk;
  NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
  uint32_t t0=micros(); while(!NRF_RADIO->EVENTS_END && (micros()-t0)<500){}
  if (NRF_RADIO->EVENTS_END){ NRF_RADIO->EVENTS_END=0;
    bool crcok=NRF_RADIO->CRCSTATUS&1; uint8_t len=rfrx[0];
    if (len && len<64){ g_rfRxCount++;
      Serial.printf("RESP#%lu crc%d len%u: ",(unsigned long)g_rfRxCount,crcok,len);
      for(uint8_t i=0;i<=len&&i<40;i++)Serial.printf("%02X ",rfrx[i]); Serial.println();
    }
  }
  NRF_RADIO->TASKS_DISABLE=1; RWAIT_DISABLED(); NRF_RADIO->EVENTS_DISABLED=0;
}

// ===================== WebUSB config channel =====================
// Browser panel (copycat_config.html) <-> firmware, binary framed.
//   host->dev:  0x01                  GET  -> reply status blob
//               0x02 <field> <value>  SET one byte field (1=mDiv 2=mFric 3=padSmooth 4=abSwap 5..8=back[0..3] 9=pollU100 10=e7b 11=relayOp 12=relaySub 13=testHaptic)
//               0x03 <mode>           switch USB mode (0=steam 1=xbox 2=switch): persist + reboot
//   dev->host:  0xA5 <len> <payload>  payload = [ver=1][mode][mDiv][mFric][padSmooth][abSwap]
//                                                [back0..3][connSlot(0xFF=none)][linkUp][f1ps_lo][f1ps_hi][pollU100][newps_lo][newps_hi][e7b][relayOp][relaySub]
#define WB_PAYLEN 23
static void webusbSendBlob(){
  if(!usb_web.connected()) return;
  bool up = (g_connSlot>=0 && (millis()-g_connReplyMs) < 300);
  uint8_t p[2+WB_PAYLEN];
  p[0]=0xA5; p[1]=WB_PAYLEN;
  p[2]=1;                          // protocol version
  p[3]=g_usbMode; p[4]=(uint8_t)g_mDiv; p[5]=(uint8_t)g_mFric; p[6]=(uint8_t)g_padSmooth; p[7]=g_abSwap;
  p[8]=g_back[0]; p[9]=g_back[1]; p[10]=g_back[2]; p[11]=g_back[3];
  p[12]=(g_connSlot>=0)?(uint8_t)g_connSlot:0xFF;
  p[13]=up?1:0;
  p[14]=(uint8_t)g_f1ps; p[15]=(uint8_t)(g_f1ps>>8);
  p[16]=(uint8_t)(g_pollUs/100);
  p[17]=(uint8_t)g_newps; p[18]=(uint8_t)(g_newps>>8);
  p[19]=g_e7b;
  p[20]=g_relayOp; p[21]=g_relaySub; p[22]=g_fwdNewOnly; p[23]=g_qos; p[24]=g_persistMode?1:0;
  usb_web.write(p,sizeof p); usb_web.flush();
}
static void webusbPoll(){
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
        bool persist=true;   // most fields persist; the rate knobs are session-only (locked default on reboot)
        switch(f){
          case 1: g_mDiv = v<4?4:v; break;
          case 2: g_mFric = v>99?99:v; break;
          case 3: g_padSmooth = v<5?5:(v>100?100:v); break;
          case 4: g_abSwap = v?1:0; break;
          case 5: case 6: case 7: case 8: g_back[f-5]=v; break;
          case 9: g_pollUs = (uint32_t)(v<1?1:v)*100; persist=false; break;   // pollU100 -> us. SESSION-ONLY: poll rate is locked; this tunes it live but reverts to 800 on reboot.
          case 10: g_e7b = v?1:0; break;                       // E7 protocol-version B-byte (experimental v1 fast)
          case 11: g_relayOp = v; break;                       // haptic-relay opcode (buzz hunt)
          case 12: g_relaySub = v; break;                      // haptic-relay sub-type (buzz hunt)
          case 13: g_testHaptic = v?v:40; break;               // inject v test haptics (0->40)
          case 14: g_fwdNewOnly = v?1:0; break;                // Steam: forward only fresh reports (dedupe)
          case 15: g_qos = v?1:0; g_hopIdx=0; g_qosBad=0; g_qosCheckMs=millis(); break;  // QoS adaptive channel hopping
          case 16: g_persistMode = v?true:false; break;   // persist last mode across reboots (else always boot Steam)
        }
        if(persist) saveCfg();
        webusbSendBlob();
      } else if(op==0x03){
        uint8_t m=buf[1]; if(m<=2){ webusbSendBlob(); usb_web.flush(); saveMode(m); delay(40); NVIC_SystemReset(); }
      }
      memmove(buf,buf+need,n-need); n-=need;
    }
  }
}

// CDC commands: l=listen, s=stop, cN=channel, p<hex>=prefix, a<8hex>=base addr, b=bonds
static void rfSerialPoll(){
  static char line[24]; static uint8_t li=0;
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n'||c=='\r'){ line[li]=0;
      if (line[0]=='l') rfListenStart();
      else if (line[0]=='B'){ g_rfBeacon=!g_rfBeacon; g_rfListen=false; Serial.printf("# beacon %s ch%u\n",g_rfBeacon?"ON":"off",g_rfCh); }
      else if (line[0]=='L'){ g_plen=strtol(line+1,0,16); Serial.printf("# plen=%02X\n",g_plen); }
      else if (line[0]=='1'){ g_s1incl=!g_s1incl; Serial.printf("# s1incl=%u\n",g_s1incl); }
      else if (line[0]=='R'){ g_rfCh=atoi(line+1); g_rfRaw=true; g_rfListen=false; g_rfBeacon=false; rfRawStart(g_rfCh); Serial.printf("# raw cap ch%u\n",g_rfCh); }
      else if (line[0]=='Z'){ g_rfRaw=true; g_rfSweep=true; g_rfListen=false; g_rfBeacon=false; g_rfCh=2; rfRawStart(g_rfCh); Serial.println("# raw SWEEP"); }
      else if (line[0]=='H'){ g_rfHost=!g_rfHost; g_rfBeacon=false; g_rfListen=false; g_rfRaw=false; g_rfSweep=false; g_rfCh=2;
        Serial.printf("# HOSTFRAME beacon %s base %02X%02X%02X%02X (slot0 uuids %02X%02X%02X%02X/%02X%02X%02X%02X)\n",
          g_rfHost?"ON":"off",g_rfBase[0],g_rfBase[1],g_rfBase[2],g_rfBase[3],
          g_slot[0].rec[0],g_slot[0].rec[1],g_slot[0].rec[2],g_slot[0].rec[3],
          g_slot[0].rec[4],g_slot[0].rec[5],g_slot[0].rec[6],g_slot[0].rec[7]); }
      else if (line[0]=='C'){ g_rfBeacon=g_rfListen=g_rfRaw=g_rfSweep=g_rfHost=false; rfRespondStart(); }
      else if (line[0]=='s'){ g_rfListen=false; g_rfBeacon=false; g_rfRaw=false; g_rfSweep=false; g_rfHost=false; g_rfRespond=false; NRF_RADIO->TASKS_DISABLE=1; Serial.println("# RF off"); }
      else if (line[0]=='x'){ uint8_t m=strtoul(line+1,0,10); if(m<=2){ Serial.printf("# switch mode %u (reboot)\n",m); delay(20); saveMode(m); delay(40); NVIC_SystemReset(); } }   // switch USB mode 0=steam 1=xbox 2=switch (lizard is automatic in steam mode)
      else if (line[0]=='c'){ g_rfCh=atoi(line+1); Serial.printf("# ch=%u\n",g_rfCh); if(g_rfListen) rfListenStart(); }
      else if (line[0]=='p'){ g_rfPrefix=strtol(line+1,0,16); Serial.printf("# prefix=%02X\n",g_rfPrefix); if(g_rfListen) rfListenStart(); }
      else if (line[0]=='i'){ g_crcinit=strtoul(line+1,0,16); Serial.printf("# crcinit=%06lX\n",(unsigned long)g_crcinit); }
      else if (line[0]=='w'){ g_whiteiv=atoi(line+1); Serial.printf("# whiteiv=%u\n",g_whiteiv); }
      else if (line[0]=='0'){ g_pcnf0=strtoul(line+1,0,16); Serial.printf("# pcnf0=%08lX\n",(unsigned long)g_pcnf0); }
      else if (line[0]=='2'){ g_pcnf1=strtoul(line+1,0,16); Serial.printf("# pcnf1=%08lX\n",(unsigned long)g_pcnf1); }
      else if (line[0]=='M'){ g_mode=strtoul(line+1,0,16); Serial.printf("# mode=%u (3=Ble1M,4=Ble2M,0/2=Nrf)\n",g_mode); }
      else if (line[0]=='I'){ g_crcinit=strtoul(line+1,0,16); Serial.printf("# crcinit=%08lX\n",(unsigned long)g_crcinit); }
      else if (line[0]=='P'){ g_crcpoly=strtoul(line+1,0,16); Serial.printf("# crcpoly=%lX\n",(unsigned long)g_crcpoly); }
      else if (line[0]=='N'){ g_crccnf=strtoul(line+1,0,16); Serial.printf("# crccnf=%X\n",g_crccnf); }
      else if (line[0]=='Y'){ g_rfCap=true; g_rfCapOne=true; g_rfReplay=g_rfHost=g_rfAuto=false; g_rfCh=2; rfCapStart(2); Serial.println("# capture ONE frame for replay..."); }
      else if (line[0]=='y'){ g_rfReplay=!g_rfReplay; g_rfCap=g_rfHost=g_rfAuto=false; Serial.printf("# REPLAY %s (%uB)\n",g_rfReplay?"ON":"off",g_replayLen); }
      else if (line[0]=='T'){ g_statlen=strtoul(line+1,0,16); g_pcnf1=0; Serial.printf("# statlen=%02X\n",g_statlen); }
      else if (line[0]=='k'){ g_connOn=!g_connOn; if(g_connOn){ g_connSt=0; g_connStep=0; g_connPoll=0; g_connF1=0; } Serial.printf("# CONN mode %s: E7-awake[00 00] -> E3+GET-report-0x45 poll on ch%u (param=%02X). F1 seen=%lu\n",g_connOn?"ON":"off",g_rfCh,g_getParam,(unsigned long)g_connF1); }
      else if (line[0]=='q'){ g_getParam = g_getParam ? 0x00 : 0x2D; Serial.printf("# GET-0x45 param=%02X\n",g_getParam); }
      else if (line[0]=='v'){ g_connVerbose=!g_connVerbose; Serial.printf("# conn verbose %s (F1 seen=%lu)\n",g_connVerbose?"ON":"off",(unsigned long)g_connF1); }
      else if (line[0]=='C'){ g_sessCh=strtoul(line+1,0,10); Serial.printf("# session channel=%u (re-pair/reconnect to apply)\n",g_sessCh); }
      else if (line[0]=='U'){ g_pollUs=strtoul(line+1,0,10); if(g_pollUs<50)g_pollUs=50; Serial.printf("# poll interval=%lu us (~%lu/s) SESSION-ONLY (locked default 800us on reboot)\n",(unsigned long)g_pollUs,(unsigned long)(1000000/g_pollUs)); }
      else if (line[0]=='E'){ g_mDiv=strtoul(line+1,0,10); if(g_mDiv<4)g_mDiv=4; saveCfg(); Serial.printf("# xbox-mouse sensitivity divisor=%d (lower=faster)\n",g_mDiv); }
      else if (line[0]=='F'){ g_mFric=strtoul(line+1,0,10); if(g_mFric>99)g_mFric=99; saveCfg(); Serial.printf("# xbox-mouse friction=%d%% (higher=more glide/momentum)\n",g_mFric); }
      else if (line[0]=='D'){ g_padSmooth=strtoul(line+1,0,10); if(g_padSmooth<5)g_padSmooth=5; if(g_padSmooth>100)g_padSmooth=100; saveCfg(); Serial.printf("# steam pad smoothing alpha=%d%% (100=off, lower=smoother/laggier)\n",g_padSmooth); }
      else if (line[0]=='W'){ g_abSwap=!g_abSwap; saveCfg(); Serial.printf("# A/B + X/Y swap %s (Nintendo layout)\n",g_abSwap?"ON":"off"); }
      else if (line[0]=='K'){ int i=line[1]-'0'; uint8_t code=strtoul(line+2,0,10); if(i>=0&&i<4){ g_back[i]=code; saveCfg(); Serial.printf("# back[%d] (%s) -> code %u  [0=none 1=A 2=B 3=X 4=Y 5=LB 6=RB 7=L3 8=R3 9=Back 10=Start 11=Guide]\n",i,(const char*[]){"L4","R4","L5","R5"}[i],code); } else Serial.println("# usage: K<0-3> <code>  (0=L4 1=R4 2=L5 3=R5)"); }
      else if (line[0]=='J'){ char* sp=0; uint8_t id=strtoul(line+1,&sp,0); uint16_t val=sp?strtoul(sp,0,0):0;  // inject SET-SETTINGS to controller: report 0x87 [id][val u16 LE]
        g_relayBuf[0]=0x87; g_relayBuf[1]=3; g_relayBuf[2]=id; g_relayBuf[3]=val&0xFF; g_relayBuf[4]=val>>8; g_relayN=5; g_relayPend=true;
        Serial.printf("# queued SET-SETTINGS id=0x%02X val=%u (relay 0x87) — watch new=/s\n",id,val); }
      else if (line[0]=='G'){ g_sniff=!g_sniff; g_sniffPh=0; g_rfHost=g_connOn=g_rfCap=g_rfAuto=g_rfReplay=false; if(g_sniff) rfSniffStart(); Serial.printf("# SESSION SNIFF %s (phase0: catch host frame on ibex/ch2; phase1 PARK ch=%s)\n",g_sniff?"ON":"off",g_sniffPark?"override":"primary"); }
      else if (line[0]=='g'){ g_sniffPark=strtoul(line+1,0,10); if(g_sniffPh==1) rfSniffStart(); Serial.printf("# sniff park channel=%u (0=learned primary)\n",g_sniffPark); }
      else if (line[0]=='V'){ g_e7b=strtoul(line+1,0,10); Serial.printf("# E7 protocol-version B-byte=%u (0=awake/slow, 1=try v1-fast; watch new=/s; revert if input stops)\n",g_e7b); }
      else if (line[0]=='O'){ g_relayOp=strtoul(line+1,0,16); Serial.printf("# relay opcode=%02X\n",g_relayOp); }
      else if (line[0]=='o'){ g_relaySub=strtoul(line+1,0,16); Serial.printf("# relay sub-type=%02X\n",g_relaySub); }
      else if (line[0]=='h'){ uint8_t nc=strtoul(line+1,0,10); Serial.printf("# HOP %u->%u (advertise on current ch, then poll new). Watch F1=/s\n",g_sessCh,nc); rfHopTo(nc); }
      else if (line[0]=='z'){ g_qos=!g_qos; g_hopIdx=0; g_qosBad=0; g_qosCheckMs=millis(); Serial.printf("# QoS adaptive channel hopping %s (candidates 18,46,76,22,68)\n",g_qos?"ON":"off"); }
      else if (line[0]=='r'){ g_rxWin=strtoul(line+1,0,10); if(g_rxWin<150)g_rxWin=150; Serial.printf("# poll RX-window=%lu us (poll rate caps ~%lu/s)\n",(unsigned long)g_rxWin,(unsigned long)(1000000/g_rxWin)); }
      else if (line[0]=='e'){ g_e3mode=strtoul(line+1,0,10); Serial.printf("# E3 poll PID mode=%u (0=fixed07, 1=cyclePID+noack1, 2=cyclePID+noack0) - watch new=/s\n",g_e3mode); }
      else if (line[0]=='t'){ uint8_t n=line[1]?strtoul(line+1,0,10):40;     // inject n test haptics (output 0x82 [01 01 F7]) over the relay
        g_testHaptic=n; Serial.printf("# test-haptic burst x%u queued (relay 0x82 01 01 F7 via op=%02X sub=%02X)\n",n,g_relayOp,g_relaySub); }
      else if (line[0]=='j'){ g_connType=strtoul(line+1,0,16); Serial.printf("# connType=%02X\n",g_connType); }
      else if (line[0]=='Q'){ g_connLen=strtoul(line+1,0,16); Serial.printf("# connLen=%02X\n",g_connLen); }
      else if (line[0]=='A'){ g_balen=strtoul(line+1,0,16); g_pcnf1=0; Serial.printf("# balen=%u\n",g_balen); }
      else if (line[0]=='a'){ uint32_t v=strtoul(line+1,0,16); g_rfBase[0]=v>>24;g_rfBase[1]=v>>16;g_rfBase[2]=v>>8;g_rfBase[3]=v; Serial.printf("# base=%08lX\n",(unsigned long)v); if(g_rfListen) rfListenStart(); }
      else if (line[0]=='b'){ for(int i=0;i<NSLOT;i++) if(g_slot[i].used){Serial.printf("slot%d ",i); for(int j=0;j<24;j++)Serial.printf("%02X",g_slot[i].rec[j]); Serial.println();} }
      else if (line[0]=='X'){ g_rfCap=!g_rfCap; g_rfHost=g_rfAuto=g_rfBeacon=g_rfListen=g_rfRaw=g_rfSweep=g_rfRespond=false; g_rfCh=2; g_capV=0; if(g_rfCap) rfCapStart(g_rfCh); Serial.printf("# CAPTURE(listen) %s ch%u on \"ibex\", cycling 1M/2M x BALEN3/4 (use cN to set ch)\n", g_rfCap?"ON":"off",g_rfCh); }
      else if (line[0]=='S'){ g_rfAuto=!g_rfAuto; g_rfHost=g_rfAuto; g_rfBeacon=g_rfListen=g_rfRaw=g_rfSweep=false; g_rfCh=2; g_cfgIdx=0; if(g_rfAuto) applyCfg(0); Serial.printf("# AUTOSWEEP %s (%u cfgs x300ms)\n", g_rfAuto?"ON":"off",(unsigned)(sizeof SWEEP/sizeof SWEEP[0])); }
      li=0;
    } else if (li<sizeof line-1) line[li++]=c;
  }
}

void setup() {
  genSerial();
  InternalFS.begin();
  loadCfg(); g_xbox = (g_usbMode != 0);   // load persisted config + decide USB presentation BEFORE registering interfaces

  // ---- CLEAN presentation for Xbox + Switch ----
  // The Adafruit core auto-adds a CDC (Serial) interface at startup, making every device a CDC composite
  // (bDeviceClass=0xEF/IAD). That breaks the clean controller personalities:
  //   * XBOX: Windows' xusb (Xbox 360) driver matches 045E:028E at the DEVICE level. In a composite the
  //     gamepad must stay MI_00 and the extra mouse must be a plain boot mouse interface.
  //   * SWITCH: a real console rejects composite devices outright; it accepts only a bare HID gamepad.
  // Fix: tear the auto CDC composite down and rebuild only the intended controller interfaces. clearConfiguration() also
  // resets bDeviceClass to 0x00 (single function). We deliberately KEEP bcdUSB at 0x0200 (no WebUSB / USB 2.1)
  // so the host never requests the WebUSB BOS — whose MS-OS-2.0 blob would otherwise tell Windows to bind
  // WinUSB to interface 0 (our gamepad). detach()/attach() forces the host to re-read the rebuilt descriptor.
  // Force a CLEAN re-enumeration on every boot / mode-switch: detach FIRST so the host drops the previous
  // identity, rebuild the descriptor, then re-attach (at the end of setup). This is done in ALL modes, not
  // just the clean ones: switching Xbox->Steam while Steam holds the device open could otherwise leave the
  // host caching the old controller identity, so the puck sometimes failed to enumerate. The detach gives the OS
  // an unambiguous disconnect to release its handles before the new descriptor comes up.
  const bool cleanDevice = (g_usbMode == 1 || g_usbMode == 2);
  USBDevice.detach(); delay(30);
  if (cleanDevice) USBDevice.clearConfiguration();   // clean modes also drop the auto CDC composite (resets bDeviceClass=0)

  // Distinct USB serial PER MODE (must be set AFTER clearConfiguration, which nulls it). Hosts cache USB
  // identity by VID:PID:serial; reusing one serial under a changing VID:PID can make a host refuse the new
  // identity. Steam keeps the exact unit serial (its pairing identity); the others get a 1-char suffix.
  if (g_usbMode == 0) { USBDevice.setSerialDescriptor(g_unit); }
  else { snprintf(g_usbSerial, sizeof g_usbSerial, "%s%c", g_unit, g_usbMode==1?'X':'N'); USBDevice.setSerialDescriptor(g_usbSerial); }
  if (g_usbMode == 1) {                // XBOX presentation: XInput vendor interface + HID boot mouse
    USBDevice.setID(0x045E, 0x028E);   // device-level 045E:028E match -> Windows xusb / SDL / Linux xpad all bind it
    USBDevice.setVersion(0x0200); USBDevice.setDeviceVersion(0x0114);   // bcdUSB 0x0200 = no BOS request (clean, no WinUSB fight)
    USBDevice.setManufacturerDescriptor("Microsoft");
    USBDevice.setProductDescriptor("Controller");
    g_xinput.setStringDescriptor("Controller"); g_xinput.begin();   // XInput vendor interface (FF/5D/01) -> MI_00
    g_mouse.setStringDescriptor("OpenPuck Mouse");
    g_mouse.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
    g_mouse.setReportDescriptor(MOUSE_HID_DESC, sizeof MOUSE_HID_DESC); g_mouse.setPollInterval(1); g_mouse.begin();
  } else if (g_usbMode == 2) {         // SWITCH presentation: a LONE HID HORIPAD/Pokkén pad (console-compatible)
    USBDevice.setID(0x0F0D, 0x0092);   // HORI Pokkén Tournament Pro Pad (whitelisted by a real Switch console)
    USBDevice.setVersion(0x0200); USBDevice.setDeviceVersion(0x0200);
    USBDevice.setManufacturerDescriptor("HORI CO.,LTD.");
    USBDevice.setProductDescriptor("POKKEN CONTROLLER");
    g_switch.enableOutEndpoint(true);  // real HORIPAD has an interrupt OUT ep (the descriptor declares an output report); the console expects the bidirectional interface
    g_switch.setReportDescriptor(SWITCH_HID_DESC, sizeof SWITCH_HID_DESC);
    g_switch.setPollInterval(8);       // 8ms (~125Hz); we stream the 8-byte report (no handshake needed)
    g_switch.begin();
  } else {                             // STEAM presentation: the 4 puck slot interfaces (desktop lizard is automatic when Steam is closed)
    USBDevice.setID(0x28DE, 0x1304);
    USBDevice.setManufacturerDescriptor("Valve Software");
    USBDevice.setProductDescriptor("Steam Controller Puck");
    for (int i = 0; i < NSLOT; i++) {
      hid[i].setReportDescriptor(PUCK_HID_DESC, sizeof PUCK_HID_DESC);
      hid[i].setReportCallback(GETCB[i], SETCB[i]);
      hid[i].setPollInterval(1);       // 1ms USB poll (was default 10ms = 100/s cap -> choppy)
      hid[i].begin();
    }
  }
  // WebUSB config panel: only in STEAM mode (the one composite mode). Xbox/Switch are clean controllers —
  // adding a vendor config interface (and the USB-2.1 BOS it forces) would re-create the exact composite that
  // stops them binding. In those modes, reconfigure over RF (back-paddle chord) or from Steam mode.
  if (g_usbMode == 0) usb_web.begin();   // no landing page -> no Chrome auto-notification
  USBDevice.attach();   // re-connect with the final descriptor (host re-reads it fresh -> deterministic enumeration)
  Serial.begin(115200);
  for (int i=0; i<300 && !USBDevice.mounted(); i++) delay(10);   // wait up to 3s for USB mount, but NEVER hang:
  loadBonds();                                                   // if a mode fails to enumerate, still run loop() so RF + the back-paddle mode chord keep working (can always switch back)
  Serial.printf("# copycat up: unit=%s board=%s, mode=%s\n", g_unit, g_board, g_usbMode==1?"XBOX(controller+mouse)":g_usbMode==2?"SWITCH(pro controller)":"STEAM(puck; auto-lizard when Steam closed)");
}

void loop() {
  if (g_dirty) { g_dirty = false; saveBonds(); }
  webusbPoll();
  swStream();        // SWITCH mode: stream 0x30 input reports once the host enabled mode 0x30
  rfSerialPoll();
  rfPoll();
  rfRespondPoll();
  rfCapPoll();
  if (g_rfCap && millis() - g_lastHop >= 90) {    // sweep DATAWHITEIV (0..127) x BALEN{3,4} on ch2, CRC-validate
    g_lastHop = millis(); g_capV = (uint8_t)(g_capV+1); rfCapStart(g_rfCh);
  }
  if (g_rfBeacon && millis() - g_lastBeacon >= 5) { g_lastBeacon = millis(); rfBeaconOnce(); }
  if (g_rfSweep && millis() - g_lastHop >= 60) { g_lastHop = millis(); g_rfCh += 2; if (g_rfCh > 80) g_rfCh = 2; rfRawStart(g_rfCh); }
  if (g_rfAuto && millis() - g_lastCfg >= 500) {   // advance to next candidate config every 500ms
    g_lastCfg = millis(); g_cfgIdx = (g_cfgIdx + 1) % (sizeof SWEEP/sizeof SWEEP[0]); applyCfg(g_cfgIdx);
  }
  if (g_rfReplay) { rfReplayOnce(); }   // bit-perfect replay of a captured real-puck frame on ch2
  // Host-frame beacon: sent continuously, INCLUDING while connected. The controller uses the periodic E1
  // (the real puck's per-hop-cycle announce) to stay synced and keep answering polls at full rate;
  // suppressing it drops the reply rate from ~210/s to ~38/s. Paused only during the post-disconnect
  // cooldown so a controller that's powering off isn't immediately re-woken/reconnected.
  if (g_rfHost && millis()-g_connCooldown > 2500) {
    bool connNow = (g_connSlot>=0 && millis()-g_connReplyMs < 300);
    // session keepalive on the clean channel: every loop while connecting (fast), every 25ms once connected
    // (every-loop beaconing also hammers ch52 and steals reply slots from the poll)
    if (millis()-g_lastSessBeacon >= (connNow ? 25u : 0u)) { g_lastSessBeacon=millis(); g_rfCh=g_sessCh; for (int s=0;s<NSLOT;s++) rfHostFrameOnce(s); }
    // discovery beacon on ch2 (where a searching controller looks): every loop when down, occasionally when up
    if (millis()-g_lastDisc >= (connNow ? 200u : 0u)) { g_lastDisc=millis(); g_rfCh=2; for (int s=0;s<NSLOT;s++) rfHostFrameOnce(s); }
  }
  if (g_connOn && millis()-g_connCooldown > 2500) { rfConnStep(); }            // connected-mode: poll controller, read input
  { static bool wasHapticLinkUp=false;
    bool up=hapticLinkUp();
    if(up && !wasHapticLinkUp) g_hapticBlockUntil = millis() + HAPTIC_RECONNECT_BLOCK_MS;   // reconnect hygiene: don't replay Steam's stale startup haptics
    wasHapticLinkUp=up;
  }
  // Legacy XInput rumble -> relay the haptic (0x82 [01 01 gain]) to the controller while the host commands it,
  // re-queued ~40/s like Steam's glide haptic so it sustains. Not active in the current HID Xbox presentation.
  if (g_usbMode==1 && g_rumble && millis()-g_rumbleMs > RUMBLE_STUCK_MS) g_rumble=0;   // watchdog: lost stop packet -> release the held rumble instead of buzzing forever
  if (g_usbMode==1 && g_rumble && !g_relayPend && g_connSlot>=0) {
    static unsigned long lastRumble=0;
    if (millis()-lastRumble>=25) { lastRumble=millis();
      uint8_t gain = g_rumble<0x30?0x30:g_rumble;   // floor so low rumble is still feelable
      g_relayBuf[0]=0x82; g_relayBuf[1]=3; g_relayBuf[2]=0x01; g_relayBuf[3]=0x01; g_relayBuf[4]=gain; g_relayPend=true;
    }
  }
  // Steam-mode haptic guard: do not synthesize haptic packets. The reconnect guard above only drops stale
  // host 0x82 traffic until the link is stable; active-session haptics are relayed verbatim.
  if (!g_xbox && g_haptic82On && millis()-g_haptic82Ms > HAPTIC_QUIET_MS) g_haptic82On=false;
  // QoS: if the current channel is degrading (crcfail+noRx), hop to the next clean candidate (conservative).
  if (g_qos && g_connSlot>=0 && millis()-g_qosCheckMs>=600) {
    uint16_t bad=g_qosBad; g_qosBad=0; g_qosCheckMs=millis();
    if (bad>20 && millis()-g_qosLastHopMs>2000) {
      for(int k=0;k<(int)sizeof g_hopCand;k++){ g_hopIdx=(g_hopIdx+1)%(sizeof g_hopCand); if(g_hopCand[g_hopIdx]!=g_sessCh) break; }
      if(Serial.availableForWrite()>60) Serial.printf("# QoS: ch%u bad=%u -> hop ch%u\n",g_sessCh,bad,g_hopCand[g_hopIdx]);
      rfHopTo(g_hopCand[g_hopIdx]); g_qosLastHopMs=millis();
    }
  }
  // USB connection presentation (like the real dongle): report 0x79 = connection state (01=disc, 02=conn),
  // edge-triggered, + periodic 0x7B status. Live-captured: this is what Steam reads to mark the controller
  // connected. Without it Steam shows disconnected even though 0x45 input is streaming.
  if(!g_xbox){ static bool usbConn=false; static unsigned long last79=0, last7B=0;   // puck-only USB presentation
    bool conn = (g_connSlot>=0 && millis()-g_connReplyMs < 300);
    if (g_connSlot>=0 && g_connSlot<NSLOT && hid[g_connSlot].ready()){
      // 0x79 connection state: on edge AND repeated every 750ms while connected (Steam can miss the edge,
      // and the 0x45 stream can starve the endpoint at the transition instant -> resend until it sticks).
      if (conn!=usbConn || (conn && millis()-last79>=750)){
        uint8_t st=conn?0x02:0x01; hid[g_connSlot].sendReport(0x79,&st,1); usbConn=conn; last79=millis();
      } else if (conn && millis()-last7B>=2000){
        static const uint8_t s7b[12]={0xF7,0x01,0x89,0x00,0x00,0x00,0x03,0x00,0xDD,0x00,0x3A,0x02};
        hid[g_connSlot].sendReport(0x7B,s7b,12); last7B=millis();
      }
    } else if(!conn) usbConn=false;
  }
  if (g_connOn && millis()-g_stMs>=1000){ g_f1ps=g_stF1; g_newps=g_stNew; if(Serial.availableForWrite()>70) Serial.printf("# stat polls=%lu/s F1=%lu/s new=%lu/s F3=%lu/s(v%d) e7b=%u crcfail=%lu noRx=%lu slot=%d\n",(unsigned long)g_stPoll,(unsigned long)g_stF1,(unsigned long)g_stNew,(unsigned long)g_stF3,(int8_t)g_connF3v,g_e7b,(unsigned long)g_stCrc,(unsigned long)g_stNoRx,g_connSlot); g_stPoll=0; g_stF1=0; g_stNew=0; g_stF3=0; g_stCrc=0; g_stNoRx=0; g_chF1[0]=g_chF1[1]=g_chF1[2]=0; g_stMs=millis(); }
  rfSniffPoll();
  // phase-1 PARKS (no hop) on the primary/override channel — re-arm RX only if it dropped (handled in rfSniffPoll via SHORTS END_START)
  if (g_rfRespond && millis() - g_lastHop >= 25) {          // scan the 3 BLE adv channels w/ matched whitening
    g_lastHop = millis();
    static const uint8_t advf[3]={2,26,80}, advw[3]={37,38,39}; static uint8_t ai=0; ai=(ai+1)%3;
    g_rfCh=advf[ai]; g_whiteiv=advw[ai];
    rfConfig(g_rfCh); rfSetAddr(g_rfBase,g_rfPrefix); NRF_RADIO->PACKETPTR=(uint32_t)rfrx;
    NRF_RADIO->SHORTS=RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_START_Msk; NRF_RADIO->EVENTS_END=0; NRF_RADIO->TASKS_RXEN=1;
  }
}
