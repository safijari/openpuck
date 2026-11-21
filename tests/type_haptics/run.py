from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
s = (root/'OpenPuck/config.cpp').read_text()
# the per-type defaults and rumbleStoreActive(), as compiled into config.cpp
slice_ = s[s.index('uint8_t g_typeLimitKnee['):s.index('void applyActiveType()')]
test = r'''#include <cassert>
#include <cstdint>
#define ET_COUNT 4
#define ET_XBOX 0
#define ET_SWITCH 1
#define ET_DS4 2
#define ET_DS5 3
#define HAPTIC_LIMIT_KNEE_DEFAULT 85u
#define HAPTIC_LIFT_DEFAULT 1
#define HAPTIC_LIFT_DS5_DEFAULT 0
#define RUMBLE_SCALE_PCT 200u
#define RUMBLE_SCALE_MIN 10u
#define RUMBLE_SCALE_MAX 500u
int applied = 0;
void applyActiveType() { applied++; }
#define AUDIO_STYLE_RUMBLE 0
#define AUDIO_STYLE_TONE 1
#define AUDIO_STYLE_SPLIT 2
#define AUDIO_STYLE_WAVE 3
uint8_t g_etype = 0;
uint8_t g_switchHd = 1;
uint16_t g_rumbleScale = RUMBLE_SCALE_PCT;
''' + slice_ + r'''
int main()
{
	// every type starts on wave; the Switch at 100% grip strength, the rest at 200%
	for (int et = 0; et < ET_COUNT; et++) {
		assert(g_typeHapticStyle[et] == AUDIO_STYLE_WAVE);
		assert(g_typeRumbleScale[et] == (et == ET_SWITCH ? 100u : 200u));
		assert(g_typeRumbleScaleDefault[et] == g_typeRumbleScale[et]);
		assert(g_typeLimitKnee[et] == 85u && g_typeHapticLift[et] == (et == ET_DS5 ? HAPTIC_LIFT_DS5_DEFAULT : HAPTIC_LIFT_DEFAULT));
	}
	// an edit to the live strength lands on the active type only
	g_etype = ET_SWITCH;
	g_rumbleScale = 300;
	rumbleStoreActive();
	assert(g_typeRumbleScale[ET_SWITCH] == 300u && g_typeRumbleScale[ET_DS5] == 200u);
	// a puck mode (no emulated type) stores nothing
	g_etype = ET_COUNT;
	g_rumbleScale = 50;
	rumbleStoreActive();
	for (int et = 0; et < ET_COUNT; et++)
		assert(g_typeRumbleScale[et] != 50u);
	// the Switch with HD rumble off keeps its own strength, starting at the 200% plain rumble needs
	g_etype = ET_SWITCH;
	g_switchHd = 0;
	assert(g_switchPlainScale == 200u);
	g_rumbleScale = 150;
	rumbleStoreActive();
	assert(g_switchPlainScale == 150u && g_typeRumbleScale[ET_SWITCH] == 300u);
	g_switchHd = 1;
	g_rumbleScale = 120;
	rumbleStoreActive();
	assert(g_switchPlainScale == 150u && g_typeRumbleScale[ET_SWITCH] == 120u);
	// the panel edits a type by index: the Switch's follows its HD rumble setting, the rest are plain
	g_switchHd = 1;
	rumbleScaleSet(ET_SWITCH, 80);
	rumbleScaleSet(ET_DS5, 400);
	assert(rumbleScaleOf(ET_SWITCH) == 80u && rumbleScaleOf(ET_DS5) == 400u && applied == 2);
	g_switchHd = 0;
	assert(rumbleScaleOf(ET_SWITCH) == g_switchPlainScale);
	rumbleScaleSet(ET_SWITCH, 250);
	assert(g_switchPlainScale == 250u && g_typeRumbleScale[ET_SWITCH] == 80u);
	// out of range clamps, and an unknown type is ignored
	rumbleScaleSet(ET_XBOX, 5000);
	assert(rumbleScaleOf(ET_XBOX) == 500u);
	rumbleScaleSet(ET_XBOX, 1);
	assert(rumbleScaleOf(ET_XBOX) == 10u);
	rumbleScaleSet(9, 100);
	assert(applied == 5);
	return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
	p = Path(td)/'test.cpp'
	p.write_text(test)
	subprocess.run(['g++', '-std=c++11', '-g', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', str(p), '-o', td+'/test'], check=True)
	subprocess.run([td+'/test'], check=True, env={'ASAN_OPTIONS': 'detect_leaks=0'})
print('per-type haptic defaults tests passed')
