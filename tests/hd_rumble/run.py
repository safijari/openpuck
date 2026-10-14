from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
s = (root/'OpenPuck/haptics.cpp').read_text()
h = (root/'OpenPuck/haptics.h').read_text()
constants = '\n'.join(h[h.index('#define RUMBLE_STYLE_NORMAL'):h.index('extern uint16_t g_rumbleScale')].splitlines())
constants += '\n' + h[h.index('#define HSIDE_LPAD'):h.index('// PCM haptic stream')]
helpers = s[s.index('static uint32_t isqrt32('):s.index('// Queue a pending test-haptic')]
m = (root/'OpenPuck/mode_switch_pro.cpp').read_text()
decoder = m[m.index('enum { HDR_AMP_MIN'):m.index('static int jcStick12(')]
head = '''#include <cassert>
#include <cstdint>
#include <vector>
#include <cmath>
#define NSLOT 4
#define MODE_SW_PRO 4
#define RUMBLE_STOP_REPS 3
uint8_t g_rumbleStyle=0,g_usbMode=4,g_rumble=1;
uint16_t g_rumbleScale=100,g_hdPadScale=100;
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
// fork: merged legacy/audio rumble path (hapticUpdateRumble)
#define RUMBLE_THROTTLE_MS 20u
uint8_t g_audioHaptics=1;
uint8_t g_switchHd=1;
uint8_t g_hapticLimitKnee=100; // 100 = limiter off: these checks are about the HD rumble path, not the knee
uint8_t g_hapticLift=0; // soft boost off, likewise; its own checks turn it on
#define ET_SWITCH 1
#define AUDIO_STYLE_RUMBLE 0
#define AUDIO_STYLE_TONE 1
#define AUDIO_STYLE_SPLIT 2
#define AUDIO_STYLE_WAVE 3
uint8_t g_typeHapticStyle[4]={3,3,3,3};
uint16_t g_legacyLow[4]={},g_legacyHigh[4]={},g_audioLow[4]={},g_audioHigh[4]={},g_lastSentLow[4]={},g_lastSentHigh[4]={};
unsigned long g_legacyMs[4]={};
struct Msg{uint8_t rid,slot;std::vector<uint8_t> p;};
std::vector<Msg> messages;
// the grip PCM stream (0x86 / 0x88) is only recorded when a test asks for it
bool keepPcm=false;
bool relayEnqueue(uint8_t rid,const uint8_t *p,uint8_t n,bool hap,uint8_t slot){
 assert(hap && slot<4);if(keepPcm || (rid!=0x86 && rid!=0x88))messages.push_back({rid,slot,{p,p+n}});return true;
}
#define PCM_SAMPLES 31u
#define PCM_FRAME_LEN (1 + 2 * PCM_SAMPLES)
#define PCM_FRAME_L(f) ((f) + 1)
#define PCM_FRAME_R(f) ((f) + 1 + PCM_SAMPLES)
#define PCM_RATE_HZ 4000u
#define RUMBLE_TEST_AMP 0x8000u
uint32_t micros(){return now*1000u;}
uint8_t ps5WaveMask(){return 0;}
void hapticPcmStart(uint8_t slot){if(keepPcm)messages.push_back({0x86,slot,{2,2,9}});}
''' + h[h.index('static inline uint8_t hapticPcmFrameLen'):h.index('// G.711')] + '''bool hapticPcmSend(uint8_t slot,uint8_t *fr,uint8_t n){assert(n && n<=31);fr[0]=n;std::vector<uint8_t> f(fr,fr+63);if(keepPcm)messages.push_back({0x88,slot,f});return true;}
uint8_t hapticUlaw(float x){return x>0.001f?1:(x<-0.001f?2:0);}
''' + h[h.index('static inline float hapticSoftLimit'):h.index('void hapticSwitchHd(')] + '''void hapticCancelPendingOn(int slot){
 for(auto &m:messages)if(m.slot==slot && ((m.rid==0x83 && int8_t(m.p[1])!=-128) || (m.rid==0x80 && m.p[0])))m.rid=0;
}
uint8_t jcBondOf(uint8_t slot){return (slot+3)%4;}
uint16_t word(const Msg&m,int off){assert(off+1<int(m.p.size()));return m.p[off]|(m.p[off+1]<<8);}
void reset(){messages.clear();keepPcm=false;for(int i=0;i<4;i++){g_hdPcmState[i]={};g_hdRumble[i]={};g_rumble80On[i]=false;g_rumble80Ms[i]=0;g_legacyLow[i]=g_legacyHigh[i]=g_audioLow[i]=g_audioHigh[i]=g_lastSentLow[i]=g_lastSentHigh[i]=0;blocked[i]=false;}now=100;g_rumbleStyle=0;g_usbMode=4;g_rumble=1;g_rumbleScale=100;g_hdPadScale=100;g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_WAVE;g_switchHd=1;USBDevice.sleep=false;}
'''
# reset needs the production type first.
reset = head[head.index('void reset()'):]
head = head[:head.index('void reset()')]
test = '''int main(){
 // HD: each side's high band is a trackpad tone (0x83, 120 ms, never 0x80); the grips get PCM (below).
 auto side=[&](size_t i){return messages[i].p[0];};auto gain=[&](size_t i){return int(int8_t(messages[i].p[1]));};
 reset();hapticSwitchPitch(0,0,20000,0,30000,160,440,160,330);assert(messages.empty());hapticHdTask();
 assert(messages.size()==2 && messages[0].rid==0x83 && messages[1].rid==0x83);
 assert(side(0)==HSIDE_LPAD && word(messages[0],2)==440 && word(messages[0],4)==120);
 assert(side(1)==HSIDE_RPAD && word(messages[1],2)==330 && gain(1)==hdToneGain(30000)+6);
 // unchanged tones refresh every 50 ms, not every call
 hapticHdTask();now+=49;hapticHdTask();assert(messages.size()==2);now++;hapticHdTask();assert(messages.size()==4 && side(2)==HSIDE_LPAD && side(3)==HSIDE_RPAD);
 // a band going quiet cuts its actuator at once with -128 dB
 now++;hapticSwitchPitch(0,0,0,0,30000,160,440,160,330);hapticHdTask();assert(messages.size()==5 && side(4)==HSIDE_LPAD && gain(4)==-128);
 // a retune waits for the 32 ms step spacing; a hit (+6 dB) goes at once
 now+=20;hapticSwitchPitch(0,0,0,0,30000,160,440,160,400);hapticHdTask();assert(messages.size()==5);
 now+=12;hapticHdTask();assert(messages.size()==6 && word(messages[5],2)==400);
 now+=1;hapticSwitchPitch(0,0,0,0,65535,160,440,160,400);hapticHdTask();assert(messages.size()==7 && gain(6)>gain(5));
 // silence cuts the pads at once; past the PCM hang it stops without sending anything (a cut to an idle pad clicks)
 hapticSwitchHd(0,0,0,0,0);hapticHdTask();assert(g_hdRumble[0].active && messages.size()==8 && side(7)==HSIDE_RPAD && gain(7)==-128);
 now+=500;hapticHdTask();assert(!g_hdRumble[0].active && messages.size()==8);
 for(auto &m:messages)assert(m.rid!=0x80 && m.p[0]<HSIDE_LGRIP);
 // an abrupt stop mid-effect still cuts both pads, twice, and trims the grip stream with 0x86 op 1
 reset();keepPcm=true;hapticSwitchHd(0,40000,40000,40000,40000);hapticHdTask();now+=8;hapticHdTask();
 {auto n=messages.size();g_rumble=0;now+=8;hapticHdTask();assert(!g_hdRumble[0].active && messages.size()==n+3);
  for(size_t i=n;i<n+2;i++)assert(messages[i].rid==0x83 && gain(i)==-128 && side(i)==HSIDE_PADS);
  assert(messages[n+2].rid==0x86 && messages[n+2].p[0]==1);}
 // rumble turned off during the silent hang: nothing is playing, so nothing is sent
 reset();keepPcm=true;hapticSwitchHd(0,40000,40000,40000,40000);hapticHdTask();
 hapticSwitchHd(0,0,0,0,0);now+=200;hapticHdTask();assert(g_hdPcmState[0].on);
 {auto n=messages.size();g_rumble=0;now+=8;hapticHdTask();assert(!g_hdRumble[0].active && !g_hdPcmState[0].on && messages.size()==n);}
 reset();g_hdPadScale=300;g_rumbleScale=200;hapticSwitchPitch(0,0,65535,0,65535,160,275,160,275);hapticHdTask();
 assert(messages.size()==2 && gain(0)==-15 && word(messages[0],2)==275 && side(0)==HSIDE_LPAD && side(1)==HSIDE_RPAD);
 reset();hapticSwitchHd(2,65535,65535,65535,65535);g_rumbleScale=500;hapticHdTask();assert(messages.size()==2 && messages[0].slot==2);
 assert(side(0)==HSIDE_LPAD && side(1)==HSIDE_RPAD && word(messages[0],2)==320 && word(messages[1],2)==320);
 now+=601;hapticHdTask();assert(!g_hdRumble[2].active);
 reset();blocked[0]=true;hapticSwitchHd(0,60000,60000,60000,60000);hapticHdTask();assert(messages.empty());
 reset();g_rumble=0;hapticSwitchHd(0,60000,60000,60000,60000);hapticHdTask();assert(messages.empty());
 reset();hapticSwitchHd(0,60000,0,0,60000);hapticHdTask();USBDevice.sleep=true;hapticHdTask();assert(!g_hdRumble[0].active);
 reset();hapticSteamRumble(32000,16000,1);hapticHdTask();assert(messages.size()==2);
 g_usbMode=1;hapticSteamRumble(1234,5678,1);assert(!g_hdRumble[1].active);
 assert(messages.back().rid==0x80 && word(messages.back(),3)==1234 && word(messages.back(),6)==5678);
 auto n=messages.size();hapticHdTask();assert(messages.size()==n);
 reset();g_usbMode=1;hapticSteamRumble(111,222,0);assert(messages.size()==1 && word(messages[0],3)==111 && word(messages[0],6)==222);
 reset();g_usbMode=1;g_rumbleStyle=1;hapticSteamRumble(111,222,0);assert(word(messages[0],3)==222 && word(messages[0],6)==222);
 reset();g_usbMode=1;g_rumbleStyle=5;hapticSteamRumble(32768,65535,0);assert(word(messages[0],3)==16384 && word(messages[0],6)==65535);
 for(int hz : {160,230,240,249,250,275,300,301,310,320,440}) {
  for(int amp : {1000,10000,65535}) {
   reset();g_hdPadScale=300;g_rumbleScale=200;
   hapticSwitchPitch(0,0,amp,0,amp,160,hz,160,hz);hapticHdTask();
   assert(messages.size()==2);int cap=hdQuietCeiling(hz);assert(cap>=-15 && cap<=-3);
   if(hz>=250 && hz<=300)assert(cap==-15);
   int requested=hdToneGain(hdScale(amp))+6;
   for(int i=0;i<2;i++){assert(gain(i)==(requested>cap?cap:requested));assert(word(messages[i],2)==hz && word(messages[i],4)==120);}
  }
 }
 assert(hdQuietCeiling(240)==-9 && hdQuietCeiling(310)==-9 && hdQuietCeiling(160)==-3 && hdQuietCeiling(275)==-15);
 for(int amp=1;amp<65536;amp++)assert(hdToneGain(amp)>=-60 && hdToneGain(amp)<=-9);
 // trackpad strength scales the pad tones (grip strength doesn't); 0 silences them
 reset();hapticSwitchPitch(0,40000,10000,0,0,160,440,160,440);hapticHdTask();assert(messages.size()==1 && side(0)==HSIDE_LPAD);
 auto padGain=gain(0);
 now+=50;g_rumbleScale=50;hapticHdTask();assert(messages.size()==2 && gain(1)==padGain);
 now+=50;g_hdPadScale=50;hapticHdTask();assert(gain(2)<padGain);
 now+=50;g_hdPadScale=0;{auto before=messages.size();hapticHdTask();assert(messages.size()==before+1 && side(before)==HSIDE_LPAD && gain(before)==-128);}
 // grips stream both bands as 0x88 PCM frames, pads keep their tones
 reset();keepPcm=true;hapticSwitchPitch(0,30000,20000,0,0,160,320,160,320);hapticHdTask();
 // the stream starts HD_PCM_LEAD (62) samples ahead, so two full frames go out with the 0x86 at once
 assert(messages.size()==4 && messages[0].rid==0x83 && messages[0].p[0]==HSIDE_LPAD && messages[1].rid==0x86 && messages[2].rid==0x88 && messages[3].rid==0x88);
 for(auto &m:messages){assert(!(m.rid==0x83 && m.p[0]>=HSIDE_LGRIP && int8_t(m.p[1])!=-128));if(m.rid==0x88)assert(m.p.size()==63 && m.p[0]==31);}
 {auto &m=messages.back();bool left=false,right=false;for(int i=1;i<32;i++)left|=m.p[i]!=0;for(int i=32;i<63;i++)right|=m.p[i]!=0;assert(left && !right);}
 // grip strength scales the stream; 0 silences it. The third frame is 3 samples, so playback starts at 65
 // buffered samples, 1 ms after the effect instead of 24 ms
 g_rumbleScale=0;now+=1;hapticHdTask();{auto &m=messages.back();assert(m.rid==0x88 && m.p[0]==3);for(int i=0;i<3;i++)assert(m.p[1+i]==0 && m.p[32+i]==0);}
 {std::vector<int> n;for(auto &m:messages)if(m.rid==0x88)n.push_back(m.p[0]);assert((n==std::vector<int>{31,31,3}));}
 now+=8;hapticHdTask();assert(messages.back().rid==0x88 && messages.back().p[0]==31);
 // the format goes out once per stream, never restated mid-stream
 for(int k=0;k<15;k++){hapticSwitchPitch(0,30000,20000,0,0,160,320,160,320);now+=100;hapticHdTask();}
 {int f=0;for(auto &m:messages)if(m.rid==0x86 && m.p[0]==2)f++;assert(f==1);}
 g_rumbleScale=100;
 // a silent gap keeps the stream alive; past the hang it just stops sending frames (no 0x86 op 1)
 hapticSwitchHd(0,0,0,0,0);now+=100;hapticHdTask();assert(g_hdPcmState[0].on && messages.back().rid==0x88);
 // an effect after silence restarts its side's bands in step, so its envelope doesn't depend on when it lands
 assert(g_hdPcmState[0].quiet[0] && g_hdPcmState[0].quiet[1]);
 hapticSwitchPitch(0,30000,20000,0,0,160,170,160,320);now+=2;hapticHdTask();
 {auto &p=g_hdPcmState[0];assert(!p.quiet[0] && p.quiet[1] && fabsf(p.phase[1]/p.phase[0]-170.0f/160.0f)<0.001f);}
 hapticSwitchHd(0,0,0,0,0);now+=100;hapticHdTask();
 now+=500;hapticHdTask();assert(!g_hdPcmState[0].on);
 for(auto &m:messages)assert(!(m.rid==0x86 && m.p[0]==1));
 // pan test: starts on the left, is on the right half a period (1 s) in, stops after 4 s
 reset();hapticTestHdPan();hapticHdTask();assert(g_hdRumble[0].bands[0]>30000 && g_hdRumble[0].bands[2]<100);
 now+=1000;hapticHdTask();assert(g_hdRumble[0].bands[0]<100 && g_hdRumble[0].bands[2]>30000 && g_hdRumble[0].bands[3]>30000);
 now+=3100;hapticHdTask();for(int k=0;k<4;k++)assert(g_hdRumble[0].bands[k]==0);
 // soft boost: off passes through; quiet passages rise 2:1 toward 20% of full scale up to the cap; loud ones
 // and near-silence are left alone; the envelope falls slowly, so the gain does not jump back between peaks
 {HapticLift lf={};assert(hapticLiftGain(&lf,0.01f)==1.0f);
  g_hapticLift=4;lf={};assert(fabsf(hapticLiftGain(&lf,0.01f)-4.0f)<0.01f);
  lf={};assert(fabsf(hapticLiftGain(&lf,0.05f)-2.0f)<0.01f);
  lf={};assert(hapticLiftGain(&lf,0.5f)==1.0f);
  lf={};assert(fabsf(hapticLiftGain(&lf,0.001f)-2.0f)<0.05f);
  g_hapticLift=1;lf={};assert(fabsf(hapticLiftGain(&lf,0.01f)-1.5f)<0.01f);
  g_hapticLift=2;lf={};assert(fabsf(hapticLiftGain(&lf,0.01f)-2.0f)<0.01f);
  lf={};hapticLiftGain(&lf,0.5f);assert(hapticLiftGain(&lf,0.0f)==1.0f && lf.env>0.49f);
  g_hapticLift=0;}
 reset();hdrBuildLevels();hdrReset(0);uint8_t b[4]={};uint16_t lo,hi;uint8_t packet[9]={};
 assert(hdrDecode(0,0,b,&lo,&hi)==0 && lo==0 && hi==0);
 uint32_t w=(1u<<30)|(1u<<25)|(3u<<20);for(int i=0;i<4;i++)b[i]=(w>>(8*i))&255;
 assert(hdrDecode(0,0,b,&lo,&hi)==65535 && lo==65535 && hi>32000 && hi<33000);
 for(int i=0;i<4;i++){b[i]=0;}assert(hdrDecode(0,0,b,&lo,&hi)==65535);
 reset();hdrReset(0);w=(1u<<30)|(1u<<25)|(1u<<20);for(int i=0;i<4;i++){packet[i+1]=(w>>(8*i))&255;packet[i+5]=packet[i+1];}
 jcRumble(0,packet,9);hapticHdTask();assert(messages[0].slot==3 && messages[0].rid==0x83);
 // the rumble style doesn't apply in Switch Pro: HD renders whatever it is set to
 reset();hdrReset(0);g_rumbleStyle=3;jcRumble(0,packet,9);hapticHdTask();assert(messages[0].rid==0x83);
 for(auto &m:messages)assert(m.rid!=0x80);
 // Absolute/relative frequency decoding and host-pitch rendering.
 reset();hdrReset(0);
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
 hapticSwitchPitch(0,0,20000,0,30000,160,440,160,110);hapticHdTask();
 assert(messages.size()==2 && word(messages[0],2)==440 && word(messages[1],2)==110);
 assert(word(messages[0],4)==120);
 now+=15;hapticHdTask();assert(messages.size()==2);
 now+=17;hapticSwitchPitch(0,0,20000,0,30000,160,550,160,140);hapticHdTask();
 assert(messages.size()==4 && word(messages[2],2)==550 && word(messages[3],2)==140);

 reset();hdrReset(0);uint32_t burst=(2u<<30)|(1u<<25)|(1u<<20);
 for(int i=0;i<4;i++){packet[1+i]=(burst>>(8*i))&255;packet[5+i]=packet[1+i];}
 jcRumble(0,packet,9);hapticHdTask();assert(messages.empty());

 // Switch haptic styles: how the grips play the bands (the pads always play the high band as tones).
 auto count=[&](uint8_t rid,int side=-1,int g=1000){int n=0;for(auto &m:messages)if(m.rid==rid && (side<0 || m.p[0]==side) && (g==1000 || int8_t(m.p[1])==g))n++;return n;};
 // tone: each side's stronger band is a grip tone at its level in dB (full scale = 0 dB), no PCM and no 0x80
 reset();keepPcm=true;g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_TONE;hapticSwitchPitch(0,65535,0,0,32768,160,320,160,320);hapticHdTask();
 assert(count(0x83,HSIDE_LGRIP,0)==1 && count(0x83,HSIDE_RGRIP,-6)==1 && count(0x80)==0 && count(0x86)==0 && count(0x88)==0);
 for(auto &m:messages)if(m.rid==0x83 && m.p[0]==HSIDE_LGRIP){assert(word(m,2)==160 && word(m,4)==120);}
 for(auto &m:messages)if(m.rid==0x83 && m.p[0]==HSIDE_RGRIP){assert(word(m,2)==320);}
 // grip strength scales the tone level: 50% is -6 dB
 reset();g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_TONE;g_rumbleScale=50;hapticSwitchPitch(0,65535,0,0,0,160,320,160,320);hapticHdTask();assert(count(0x83,HSIDE_LGRIP,-6)==1);
 // silence cuts the grips once as the band goes quiet and stops with both-grip cuts, not pad cuts
 reset();g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_TONE;hapticSwitchPitch(0,65535,0,0,0,160,320,160,320);hapticHdTask();
 hapticSwitchHd(0,0,0,0,0);now+=1;hapticHdTask();assert(!g_hdRumble[0].active && count(0x83,HSIDE_GRIPS,-128)==2 && count(0x83,HSIDE_PADS,-128)==0);
 // split: the low band is 0x80 rumble, the high band a grip tone
 reset();g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_SPLIT;hapticSwitchPitch(0,40000,20000,0,0,160,320,160,320);hapticHdTask();
 assert(count(0x83,HSIDE_LGRIP,-10)==1 && count(0x80)==1 && messages.back().rid==0x80 && word(messages.back(),3)==40000 && word(messages.back(),6)==0);
 hapticSwitchHd(0,0,0,0,0);now+=1;hapticHdTask();{int stops=0;for(auto &m:messages)if(m.rid==0x80 && m.p[0]==0)stops++;assert(stops==RUMBLE_STOP_REPS && !g_hdRumble[0].active);}
 // rumble: the stronger band of each side as 0x80, no grip tones
 reset();g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_RUMBLE;hapticSwitchPitch(0,10000,50000,0,0,160,320,160,320);hapticHdTask();
 assert(count(0x80)==1 && word(messages.back(),3)==50000 && count(0x83,HSIDE_LGRIP)==0 && count(0x83,HSIDE_RGRIP)==0);
 // leaving rumble for wave while it sounds stops the 0x80
 g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_WAVE;now+=20;hapticHdTask();{int stops=0;for(auto &m:messages)if(m.rid==0x80 && m.p[0]==0)stops++;assert(stops==RUMBLE_STOP_REPS);}
 // leaving wave for tone while the stream plays trims it with 0x86 op 1
 reset();keepPcm=true;hapticSwitchPitch(0,30000,0,0,0,160,320,160,320);hapticHdTask();now+=8;hapticHdTask();assert(g_hdPcmState[0].on);
 g_typeHapticStyle[ET_SWITCH]=AUDIO_STYLE_TONE;hapticHdTask();assert(!g_hdPcmState[0].on);{bool off=false;for(auto &m:messages)off|=m.rid==0x86 && m.p[0]==1;assert(off);}

 // HD rumble off: the console's rumble plays as plain 0x80 motor rumble (each side's peak), never HD tones or PCM
 reset();g_switchHd=0;hdrReset(0);
 {uint32_t hdw=(1u<<30)|(1u<<25)|(1u<<20);for(int i=0;i<4;i++){packet[i+1]=(hdw>>(8*i))&255;packet[i+5]=packet[i+1];}
  jcRumble(0,packet,9);hapticHdTask();assert(!g_hdRumble[0].active && count(0x83)==0 && count(0x86)==0 && count(0x88)==0);
  assert(count(0x80)==1 && messages[0].slot==3 && messages[0].p[0]==4);
  // an unchanged frame isn't relayed again
  jcRumble(0,packet,9);assert(count(0x80)==1);}
 reset();g_switchHd=0;hapticSteamRumble(32000,16000,1);hapticHdTask();assert(count(0x83)==0 && count(0x80)==1 && word(messages[0],3)==32000 && word(messages[0],6)==16000);
 // turning it off mid-effect stops the HD pads
 reset();hapticSwitchPitch(0,0,20000,0,30000,160,440,160,330);hapticHdTask();assert(g_hdRumble[0].active);
 g_switchHd=0;hapticHdTask();assert(!g_hdRumble[0].active && count(0x83,HSIDE_PADS,-128)==2);
 // and back on, the first frame after plain rumble still goes out
 reset();g_switchHd=0;hdrReset(0);{uint32_t hdw=(1u<<30)|(1u<<25)|(1u<<20);for(int i=0;i<4;i++){packet[i+1]=(hdw>>(8*i))&255;packet[i+5]=packet[i+1];}
  jcRumble(0,packet,9);g_switchHd=1;jcRumble(0,packet,9);g_switchHd=0;auto n=count(0x80);jcRumble(0,packet,9);assert(count(0x80)==n+1);}

 uint32_t rng=7;hdrReset(0);
 for(int n=0;n<10000;n++){rng=rng*1664525u+1013904223u;for(int i=0;i<4;i++)b[i]=(rng>>(8*i))&255;hdrDecode(0,0,b,&lo,&hi);}
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(head+constants+'\n'+helpers+decoder+reset+test)
 subprocess.run(['g++','-std=c++11','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p),'-o',td+'/test'],check=True)
 subprocess.run([td+'/test'],check=True,env={'ASAN_OPTIONS':'detect_leaks=0'})
print('HD rumble rendering, stop, transition and band-decoder tests passed')
