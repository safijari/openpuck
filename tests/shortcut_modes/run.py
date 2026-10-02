from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2];s=(root/'OpenPuck/rf_link.cpp').read_text()
a=s.index('static uint8_t g_shortcutMode');b=s.index('// TX one connected packet',a)
head='''#include <stdint.h>
#include <cassert>
#define SHORTCUT_FEEDBACK 8
#define SHORTCUT_ENABLED 32
uint8_t g_shortcutFlags=63;
uint8_t g_usbMode=4;unsigned long now=100;
unsigned long millis(){return now;}
bool modeValid(uint8_t m){return m<11;}
struct {bool sleep=false;bool suspended(){return sleep;}} USBDevice;
int pulses=0,reboots=0;uint8_t target=255;
void hapticShortcutFeedback(uint8_t slot,uint8_t count){assert(slot<4 && count==1);++pulses;}
void modeSwitchReboot(uint8_t m){++reboots;target=m;}
'''
body='''int main(){
 shortcutModeRequest(1,0);assert(pulses==1 && !reboots);shortcutModeTask();assert(!reboots);
 now=449;shortcutModeTask();assert(!reboots);now=450;shortcutModeTask();assert(reboots==1 && target==1);shortcutModeTask();assert(reboots==1);
 shortcutModeRequest(2,0);now+=100;shortcutModeRequest(4,0);now+=500;shortcutModeTask();assert(reboots==1);
 shortcutModeRequest(3,1);USBDevice.sleep=true;now+=350;shortcutModeTask();assert(reboots==1);USBDevice.sleep=false;
 int before=pulses;shortcutModeRequest(255,0);assert(pulses==before);shortcutModeRequest(4,0);assert(pulses==before+1 && g_shortcutMode==255);
 g_shortcutFlags=32;shortcutModeRequest(2,0);shortcutModeTask();assert(reboots==2 && target==2);g_shortcutFlags=63;shortcutModeRequest(3,0);g_shortcutFlags=0;now+=500;shortcutModeTask();assert(reboots==2);g_shortcutFlags=63;
 now=0xfffffff0u;shortcutModeRequest(5,0);now+=350;shortcutModeTask();assert(reboots==3 && target==5);
}'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td)/'test.cpp';p.write_text(head+s[a:b]+body)
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p),'-o',td+'/test'],check=True)
 subprocess.run([td+'/test'],check=True,env={'ASAN_OPTIONS':'detect_leaks=0'})
print('Nonblocking shortcut mode transition and feedback tests passed')
