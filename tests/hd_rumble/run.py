from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
s = (root/'OpenPuck/haptics.cpp').read_text()
h = (root/'OpenPuck/haptics.h').read_text()
constants = '\n'.join(h[h.index('#define RUMBLE_STYLE_NORMAL'):h.index('extern uint16_t g_rumbleScale')].splitlines())
helpers = s[s.index('static uint32_t isqrt32('):s.index('// Queue a pending test-haptic')]
m = (root/'OpenPuck/mode_switch_pro.cpp').read_text()
decoder = m[m.index('enum { HDR_AMP_MIN'):m.index('static int jcStick12(')]
head = '''#include <cassert>
#include <cstdint>
#include <vector>
#include <cmath>
#define NSLOT 4
#define SHORTCUT_FEEDBACK 8
uint8_t g_shortcutFlags=63;
#define MODE_SW_PRO 4
#define RUMBLE_STOP_REPS 3
uint8_t g_rumbleStyle=8,g_usbMode=4,g_rumble=1;
uint16_t g_rumbleScale=100,g_hdPadScale=100;
bool hapticShortcutFeedbackActive(uint8_t slot);
bool g_hapticRelay=true;
uint32_t now=0;
unsigned long millis(){return now;}
uint32_t __get_PRIMASK(){return 0;}
void __disable_irq(){}
void __set_PRIMASK(uint32_t){}
bool blocked[4]={};
bool haptic82Blocked(int s){return blocked[s];}
bool hapticLinkUp(int s){return !blocked[s];}
struct {bool sleep=false;bool suspended(){return sleep;}} USBDevice;
unsigned long g_rumble80Ms[4]={};bool g_rumble80On[4]={};
struct Msg{uint8_t rid,slot;std::vector<uint8_t> p;};
std::vector<Msg> messages;
bool relayEnqueue(uint8_t rid,const uint8_t *p,uint8_t n,bool hap,uint8_t slot){
 assert(hap && slot<4);messages.push_back({rid,slot,{p,p+n}});return true;
}
void hapticCancelPendingOn(int slot){
 for(auto &m:messages)if(m.slot==slot && (m.rid==0x83 || (m.rid==0x81 && (m.p[5]||m.p[6])) || (m.rid==0x80 && m.p[0])))m.rid=0;
}
uint8_t jcBondOf(uint8_t slot){return (slot+3)%4;}
uint16_t word(const Msg&m,int off){assert(off+1<int(m.p.size()));return m.p[off]|(m.p[off+1]<<8);}
void reset(){messages.clear();for(int i=0;i<4;i++){g_hdRumble[i]={};g_shortcutFeedback[i]={};g_rumble80On[i]=false;blocked[i]=false;}now=100;g_rumbleStyle=8;g_usbMode=4;g_rumble=1;g_rumbleScale=100;g_hdPadScale=100;USBDevice.sleep=false;}
'''
# reset needs the production type first.
reset = head[head.index('void reset()'):]
head = head[:head.index('void reset()')]
test = '''int main(){
 reset();hapticSwitchPitch(0,0,20000,30000,0,160,440,110,320);assert(messages.empty());hapticHdTask();
 assert(messages.size()==3 && messages[0].rid==0x80 && messages[1].rid==0x83 && messages[2].rid==0x83);
 assert(word(messages[1],2)==440 && word(messages[2],2)==110 && word(messages[1],4)==20);
 hapticHdTask();assert(messages.size()==3);now+=15;hapticHdTask();assert(messages.size()==3);now++;hapticHdTask();assert(messages.size()==6);
 now++;hapticSwitchPitch(0,0,0,30000,0,160,440,110,320);hapticHdTask();assert(messages.size()==7 && messages.back().rid==0x82 && messages.back().p[0]==1);
 hapticSwitchHd(0,0,0,0,0);hapticHdTask();assert(!g_hdRumble[0].active && word(messages.back(),1)==0);
 reset();g_hdPadScale=300;g_rumbleScale=200;hapticSwitchPitch(0,0,65535,0,65535,160,275,160,275);hapticHdTask();
 assert(int8_t(messages[1].p[1])==-15 && word(messages[1],2)==275 && word(messages[0],3)==65535);
 reset();hapticSwitchHd(2,65535,65535,65535,65535);g_rumbleScale=500;hapticHdTask();assert(messages[0].slot==2 && word(messages[0],1)==65535);
 now+=601;hapticHdTask();assert(!g_hdRumble[2].active);
 reset();blocked[0]=true;hapticSwitchHd(0,60000,60000,60000,60000);hapticHdTask();assert(messages.empty());
 reset();g_rumble=0;hapticSwitchHd(0,60000,60000,60000,60000);hapticHdTask();assert(messages.empty());
 reset();hapticSwitchHd(0,60000,0,0,60000);hapticHdTask();USBDevice.sleep=true;hapticHdTask();assert(!g_hdRumble[0].active);
 reset();hapticSteamRumble(32000,16000,1);hapticHdTask();assert(messages.size()==3);
 g_rumbleStyle=0;hapticSteamRumble(1234,5678,1);assert(!g_hdRumble[1].active);
 assert(messages.back().rid==0x80 && word(messages.back(),3)==1234 && word(messages.back(),6)==5678);
 auto n=messages.size();hapticHdTask();assert(messages.size()==n);
 reset();g_usbMode=1;hapticSteamRumble(111,222,0);assert(messages.size()==1 && word(messages[0],3)==111 && word(messages[0],6)==222);
 reset();g_rumbleStyle=1;hapticSteamRumble(111,222,0);assert(word(messages[0],3)==222 && word(messages[0],6)==222);
 reset();g_rumbleStyle=5;hapticSteamRumble(32768,65535,0);assert(word(messages[0],3)==16384 && word(messages[0],6)==65535);
 for(int hz : {160,230,240,249,250,275,300,301,310,320,440}) {
  for(int amp : {1000,10000,65535}) {
   reset();g_hdPadScale=300;g_rumbleScale=200;
   hapticSwitchPitch(0,0,amp,0,amp,160,hz,160,hz);hapticHdTask();
   assert(messages.size()==3);int cap=hdQuietCeiling(hz);assert(cap>=-15 && cap<=-3);
   if(hz>=250 && hz<=300)assert(cap==-15);
   int requested=hdToneGain(hdScale(amp))+6;
   for(int i=1;i<=2;i++){assert(int8_t(messages[i].p[1])==(requested>cap?cap:requested));assert(word(messages[i],2)==hz && word(messages[i],4)==20);}
  }
 }
 assert(hdQuietCeiling(240)==-9 && hdQuietCeiling(310)==-9 && hdQuietCeiling(160)==-3 && hdQuietCeiling(275)==-15);
 for(int amp=1;amp<65536;amp++)assert(hdToneGain(amp)>=-60 && hdToneGain(amp)<=-9);
 reset();g_hdPadScale=300;g_rumbleScale=200;hapticSwitchPitch(0,0,10000,0,10000,160,440,160,440);hapticHdTask();auto padGain=messages[1].p[1];auto grip=messages[0].p;
 now+=16;g_rumbleScale=100;hapticHdTask();assert(messages[4].p[1]==padGain && messages[3].p!=grip);
 now+=16;g_hdPadScale=50;hapticHdTask();assert(int8_t(messages[7].p[1])<int8_t(padGain));
 now+=16;g_hdPadScale=0;hapticHdTask();assert(messages.back().rid==0x80);
 reset();hdrBuildLevels();hdrReset(0);uint8_t b[4]={};uint16_t lo,hi;uint8_t packet[9]={};
 assert(hdrDecode(0,0,b,&lo,&hi)==0 && lo==0 && hi==0);
 uint32_t w=(1u<<30)|(1u<<25)|(3u<<20);for(int i=0;i<4;i++)b[i]=(w>>(8*i))&255;
 assert(hdrDecode(0,0,b,&lo,&hi)==65535 && lo==65535 && hi>32000 && hi<33000);
 for(int i=0;i<4;i++){b[i]=0;}assert(hdrDecode(0,0,b,&lo,&hi)==65535);
 reset();hdrReset(0);w=(1u<<30)|(1u<<25)|(1u<<20);for(int i=0;i<4;i++){packet[i+1]=(w>>(8*i))&255;packet[i+5]=packet[i+1];}
 jcRumble(0,packet,9);hapticHdTask();assert(messages[0].slot==3 && messages[1].rid==0x83);
 g_rumbleStyle=0;jcRumble(0,packet,9);assert(!g_hdRumble[3].active && word(messages.back(),3)==65535 && word(messages.back(),6)==65535);
 // Absolute/relative frequency decoding and host-pitch rendering.
 reset();hdrReset(0);g_rumbleStyle=8;
 assert(hdrFreq5(12,0)==-12 && hdrFreq5(16,0)==12);
 assert(hdrFreq5(17,63)==64 && hdrFreq5(17,64)==64);
 assert(hdrFreq5(31,-64)==-64 && hdrFreq5(0,32)==0);
 for(int code=17;code<32;code++)assert(hdrFreq5(code,0)==((code-17)%3==0?1:((code-17)%3==2?-1:0)));
 auto setword=[&](uint32_t value){for(int i=0;i<4;i++)b[i]=(value>>(8*i))&255;};
 setword((1u<<30)|(127u<<23)|(64u<<16)|(127u<<9)|(96u<<2));
 hdrDecode(0,0,b,&lo,&hi);assert(g_hdrState[0][0].lf==0 && g_hdrState[0][0].hf==32);
 assert(g_hdrFrequency[0][64]==160 && g_hdrFrequency[1][96]==640);
 setword((2u<<30)|(127u<<23)|(24u<<18)|(24u<<13)|(24u<<8)|(32u<<1)|1u);
 hdrDecode(0,0,b,&lo,&hi);assert(g_hdrState[0][0].hf==-32 && g_hdrState[0][0].lf==0);
 setword((1u<<30)|(80u<<23)|(24u<<18)|(24u<<13)|(24u<<8)|(24u<<3)|7u);
 hdrDecode(0,0,b,&lo,&hi);assert(g_hdrState[0][0].hf==16);
 hapticSwitchPitch(0,0,20000,30000,0,160,440,110,320);hapticHdTask();
 assert(messages.size()==3 && word(messages[1],2)==440 && word(messages[2],2)==110);
 assert(word(messages[1],4)==20 && g_rumbleStyle==8);
 now+=15;hapticHdTask();assert(messages.size()==3);
 now++;hapticSwitchPitch(0,0,20000,30000,0,160,550,140,320);hapticHdTask();
 assert(messages.size()==6 && word(messages[4],2)==550 && word(messages[5],2)==140);

 reset();hdrReset(0);uint32_t burst=(2u<<30)|(1u<<25)|(1u<<20);
 for(int i=0;i<4;i++){packet[1+i]=(burst>>(8*i))&255;packet[5+i]=packet[1+i];}
 jcRumble(0,packet,9);hapticHdTask();assert(messages.empty());
 // Confirmation uses both pads, never grips, and exact finite 250 ms tones.
 reset();hapticShortcutFeedback(0,3);messages.clear();hapticShortcutFeedbackTask();
 assert(messages.size()==2);
 for(auto &m:messages)assert(m.rid==0x83 && word(m,4)==250 && int8_t(m.p[1])==-18 && word(m,2)==160);
 assert(messages[0].p[0]==1 && messages[1].p[0]==2);
 now+=249;hapticShortcutFeedbackTask();assert(messages.size()==2);
 now++;hapticShortcutFeedbackTask();assert(messages.size()==3 && messages.back().rid==0x82);
 now+=150;hapticShortcutFeedbackTask();assert(messages.size()==5);
 now+=400;hapticShortcutFeedbackTask();assert(messages.size()==7);
 now+=250;hapticShortcutFeedbackTask();assert(!hapticShortcutFeedbackActive(0));
 for(auto &m:messages)assert(m.rid!=0x80);
 reset();hapticShortcutFeedback(0,2);messages.clear();
 hapticSwitchHd(0,50000,50000,50000,50000);hapticHdTask();hapticShortcutFeedbackTask();
 assert(messages.size()==3 && messages[0].rid==0x80 && word(messages[1],4)==250);
 hapticSwitchHd(0,0,0,0,0);hapticHdTask();assert(hapticShortcutFeedbackActive(0));
 assert(messages[1].rid==0x83);
 blocked[0]=true;hapticShortcutFeedbackTask();assert(!hapticShortcutFeedbackActive(0));
 reset();g_shortcutFlags&=~SHORTCUT_FEEDBACK;hapticShortcutFeedback(0,1);assert(!hapticShortcutFeedbackActive(0));g_shortcutFlags=63;

 uint32_t rng=7;hdrReset(0);
 for(int n=0;n<10000;n++){rng=rng*1664525u+1013904223u;for(int i=0;i<4;i++)b[i]=(rng>>(8*i))&255;hdrDecode(0,0,b,&lo,&hi);}
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(head+constants+'\n'+helpers+decoder+reset+test)
 subprocess.run(['g++','-std=c++11','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p),'-o',td+'/test'],check=True)
 subprocess.run([td+'/test'],check=True,env={'ASAN_OPTIONS':'detect_leaks=0'})
print('HD rumble rendering, stop, transition and band-decoder tests passed')
