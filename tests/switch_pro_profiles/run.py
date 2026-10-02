from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'OpenPuck/config.cpp').read_text()
a=s.index('uint8_t *swProfileBack(');b=s.index('\nvoid applyActiveType()',a)
head='''#include <stdint.h>
#include <cassert>
#include <initializer_list>
#include "triton.h"
#define NSLOT 4
#define MODE_SW_PRO 4
#define SW_PROFILE_COUNT 7
struct SwProfiles { uint8_t enabled, active, back[4][4], chord[7], extraBack[3][4]; };
SwProfiles g_swProfiles = {};
uint8_t g_usbMode=MODE_SW_PRO,g_shortcutFlags=63;
uint8_t g_rumblePresets[3]={5,0,8},g_rumbleSlot=2,g_strengthSlots[2]={255,255};
uint16_t g_strengthSteps[2][3]={{200,300,500},{200,300,500}};
int modeRequests=0;
void shortcutModeRequest(uint8_t mode,uint8_t slot){assert(mode==MODE_SW_PRO && slot<4);++modeRequests;}
uint8_t g_rumbleStyle=8;
uint16_t g_hdPadScale=100,g_rumbleScale=200;
int confirmations=0;uint8_t confirmationSlot=0;
void hapticShortcutFeedback(uint8_t slot,uint8_t count){confirmations=count;confirmationSlot=slot;}
#define RUMBLE_STYLE_MAX 8
#define RUMBLE_STYLE_HD 8
int applied=0;
void applyActiveType(){++applied;}
'''
ma=s.index('static void swProfilesLoad(');mb=s.index('\nvoid loadCfg()',ma)
head += '#include <cstring>\n#define PS_DPAD_TOUCH 3\n#define PS_DPAD_CLICK 4\nuint8_t g_padStick[2]={};\n'
u=(root/'OpenPuck/gamepad_util.cpp').read_text()
ua=u.index('uint32_t padDpadButtons(');ub=u.index('\nvoid slotSticks(',ua)

m=(root/'OpenPuck/mode_switch_pro.cpp').read_text()
ha=m.index('static void swDpadClickFeedback(');hb=m.index('\nvoid SwitchProController::task()',ha)
head += '''uint8_t g_swDpadHaptics=1,g_padHaptics=1,g_swQamSelect=18;
bool blocked=false;
int pulses=0;
uint8_t emittedSide=0, emittedBond=0;
bool haptic82Blocked(int){return blocked;}
bool relayEnqueue(uint8_t rid,const uint8_t *p,uint8_t n,bool haptic,uint8_t slot){
 assert(rid==0x82 && n==3 && haptic && p[1]==2 && p[2]==0xF7);
 ++pulses;emittedSide=p[0];emittedBond=slot;return true;
}
'''
ta=s.index('\tif (c.hdPadScale2 <= 250)');tb=s.index('\t// resolve the active emulated type',ta)
head += 'struct Tail {uint8_t hdPadScale2,reservedRumbleStyles[2],reservedWaveform,reservedWavePresets[2],reservedWaveThird,reservedWaveSlot,swQamSelect,shortcutFlags,rumblePresets[3],strengthSteps[2][3],strengthSlots[2],rumbleSlot;};\nvoid loadTail(Tail c){\n'+s[ta:tb]+'}\n'
body='''int main(){
 Tail old;memset(&old,255,sizeof old);loadTail(old);assert(g_rumblePresets[2]==8 && g_shortcutFlags==61);
 Tail savedTail=old;savedTail.hdPadScale2=150;savedTail.shortcutFlags=62;savedTail.rumblePresets[0]=0;savedTail.rumblePresets[1]=5;savedTail.rumblePresets[2]=8;savedTail.rumbleSlot=2;
 savedTail.strengthSteps[0][0]=50;savedTail.strengthSteps[0][1]=150;savedTail.strengthSteps[0][2]=250;
 loadTail(savedTail);assert(g_hdPadScale==300 && g_strengthSteps[0][0]==100 && g_shortcutFlags==62);
 assert(!shortcutHeld(TB_QAM|TB_A));assert(shortcutHeld(CHORD_BACK4|TB_A));assert(shortcutHostButtons(CHORD_BACK4|TB_A)==0);
 g_shortcutFlags=63;assert(shortcutHostButtons(TB_QAM|TB_A|TB_L4)==TB_L4);
 g_shortcutFlags=0;assert(shortcutHostButtons(TB_QAM|TB_A)==(TB_QAM|TB_A));
 g_shortcutFlags=63;g_rumbleStyle=8;g_rumbleSlot=2;
 for(int cycle=0;cycle<3;cycle++){rumbleChord(0,0);for(int i=0;i<12;i++)assert(rumbleChord(0,TB_QAM|TB_DLF));assert(g_rumbleSlot==cycle && g_rumbleStyle==g_rumblePresets[cycle] && confirmations==cycle+1);for(int i=0;i<100;i++)rumbleChord(0,TB_QAM|TB_DLF);assert(g_rumbleSlot==cycle);}
 g_strengthSteps[0][0]=100;g_strengthSteps[0][1]=300;g_strengthSteps[0][2]=500;g_hdPadScale=300;g_strengthSlots[0]=255;
 for(int cycle=0;cycle<3;cycle++){rumbleChord(0,0);for(int i=0;i<12;i++)rumbleChord(0,TB_QAM|TB_DUP);int index=(cycle+2)%3;assert(g_hdPadScale==g_strengthSteps[0][index] && confirmations==index+1);}
 g_strengthSteps[1][0]=50;g_strengthSteps[1][1]=200;g_strengthSteps[1][2]=400;g_rumbleScale=200;g_strengthSlots[1]=255;
 rumbleChord(0,0);for(int i=0;i<12;i++)rumbleChord(0,TB_QAM|TB_DDN);assert(g_rumbleScale==400 && confirmations==3);
 g_strengthSteps[0][0]=g_strengthSteps[0][1]=g_strengthSteps[0][2]=300;g_hdPadScale=300;g_strengthSlots[0]=0;
 for(int cycle=0;cycle<3;cycle++){rumbleChord(0,0);for(int i=0;i<12;i++)rumbleChord(0,TB_QAM|TB_DUP);assert(confirmations==(cycle+1)%3+1);}
 assert(!rumbleChord(0,TB_QAM|TB_MENU|TB_DLF));assert(!rumbleChord(0,TB_QAM|TB_DLF|TB_DUP));
 g_shortcutFlags&=~SHORTCUT_HAPTICS;assert(!rumbleChord(0,TB_QAM|TB_DLF));
 uint8_t defaults[]={1,5,19,20};SwProfiles saved;memset(&saved,255,sizeof saved);swProfilesLoad(saved,defaults);assert(!g_swProfiles.enabled);
 for(int p=0;p<7;p++)assert(memcmp(swProfileBack(p),defaults,4)==0);
 saved=g_swProfiles;saved.enabled=1;saved.chord[0]=2;saved.chord[3]=5;swProfilesLoad(saved,defaults);
 for(int i=0;i<12;i++){assert(swProfileChord(0,TB_QAM|TB_B));}assert(g_swProfiles.active==1 && modeRequests==1);
 for(int i=0;i<100;i++){swProfileChord(0,TB_QAM|TB_B);}assert(modeRequests==1);
 swProfileChord(0,0);g_usbMode=1;for(int i=0;i<12;i++)swProfileChord(0,TB_QAM|TB_DLF);assert(g_swProfiles.active==4 && modeRequests==2);
 g_shortcutFlags&=~SHORTCUT_PROFILES;assert(!swProfileChord(0,TB_QAM|TB_B));
 g_shortcutFlags=63;g_usbMode=MODE_SW_PRO;assert(!swProfileChord(0,TB_QAM|TB_DLF));
 int prior=confirmations;captureFeedbackChord(0,0);captureFeedbackChord(0,TB_QAM|TB_MENU);assert(confirmations==1);confirmations=4;captureFeedbackChord(0,TB_QAM|TB_MENU);assert(confirmations==4);captureFeedbackChord(0,0);captureFeedbackChord(0,TB_QAM|TB_MENU);assert(confirmations==1);(void)prior;
 uint32_t buttons=TB_QAM|TB_MENU;assert(switchSelectShortcut(0,buttons) && !(buttons&TB_MENU));buttons=TB_MENU;assert(!switchSelectShortcut(0,buttons) && !buttons);buttons=0;switchSelectShortcut(0,buttons);buttons=TB_MENU;assert(!switchSelectShortcut(0,buttons) && buttons==TB_MENU);
 PuckInput in={};g_padStick[0]=3;
 in.buttons=TB_LPADT;in.lpx=20000;assert(padDpadButtons(in)==TB_DRT);
 in.lpx=-20000;assert(padDpadButtons(in)==TB_DLF);
 in.lpx=0;in.lpy=20000;assert(padDpadButtons(in)==TB_DUP);
 in.lpy=-20000;assert(padDpadButtons(in)==TB_DDN);
 in.lpx=20000;in.lpy=20000;assert(padDpadButtons(in)==(TB_DUP|TB_DRT));
 in.lpx=1000;in.lpy=1000;assert(!padDpadButtons(in));
 in.lpx=20000;in.buttons=0;assert(!padDpadButtons(in));
 in.buttons=TB_LPADT;g_padStick[0]=4;assert(!padDpadButtons(in));
 in.buttons|=TB_LPADC;assert(padDpadButtons(in)==TB_DRT);
 in.buttons|=TB_QAM;assert(!padDpadButtons(in));
 g_padStick[0]=1;in.buttons=TB_LPADT;assert(!padDpadButtons(in));
 g_padStick[0]=0;g_padStick[1]=3;in.buttons=TB_RPADT;in.rpx=-20000;assert(padDpadButtons(in)==TB_DLF);
 in.rpx=-32768;in.rpy=-32768;assert(padDpadButtons(in)==(TB_DLF|TB_DDN));
 g_padStick[0]=3;in.buttons|=TB_LPADT;in.lpx=20000;in.lpy=0;in.rpy=0;assert(!padDpadButtons(in));
 g_padStick[0]=3;g_padStick[1]=4;
 swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==1 && emittedSide==1 && emittedBond==3);
 swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==1);
 swDpadClickFeedback(3,0);swDpadClickFeedback(3,TB_RPADT|TB_RPADC);assert(pulses==2 && emittedSide==2);
 swDpadClickFeedback(3,0);swDpadClickFeedback(3,TB_LPADT|TB_LPADC|TB_RPADT|TB_RPADC);assert(pulses==3 && emittedSide==3);
 swDpadClickFeedback(3,0);g_swDpadHaptics=0;swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==3);
 g_swDpadHaptics=1;swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==3);
 swDpadClickFeedback(3,0);blocked=true;swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==3);
 blocked=false;swDpadClickFeedback(3,0);swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==4);
 swDpadClickFeedback(3,0);swDpadClickFeedback(3,TB_LPADT|TB_LPADC|TB_QAM);assert(pulses==4);
 swDpadClickFeedback(3,0);g_padHaptics=0;swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==5);
 g_padHaptics=1;swDpadClickFeedback(3,0);g_padStick[0]=1;swDpadClickFeedback(3,TB_LPADT|TB_LPADC);assert(pulses==5);
 assert(sizeof(SwProfiles)==37);
 memset(saved.extraBack,255,12);saved.chord[0]=1;
 swProfilesLoad(saved,defaults);
 assert(g_swProfiles.enabled==1);
 for(int p=4;p<7;p++) assert(memcmp(swProfileBack(p),defaults,4)==0);
}'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(head+s[a:b]+s[ma:mb]+u[ua:ub]+m[ha:hb]+body)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(root/'OpenPuck'),str(p),'-o',td+'/test'],check=True)
 subprocess.run([td+'/test'],check=True,env={'ASAN_OPTIONS':'detect_leaks=0'})
print('Switch Pro profile shortcut tests passed')
