from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
h = (root/'OpenPuck/haptics.h').read_text()

# the shipped expander, driven the way the Wave path drives it: one peak per 4 kHz sample
exp = h[h.index('#define WAVE_EXP_LO'):h.index('void hapticSwitchHd(')]

src = '''#include <cassert>
#include <cmath>
#include <cstdio>
''' + exp + '''
static float settle(float peak)
{
	WaveExpand s = {};
	float g = 1;
	for (int i = 0; i < 4000; i++)
		g = waveExpandGain(&s, peak);
	return g;
}

int main()
{
	// below the knee the gain is exactly 1, even for silence
	assert(settle(0.0f) == 1.0f);
	assert(settle(0.03f) == 1.0f);
	assert(settle(WAVE_EXP_LO) == 1.0f);
	// it rises with the level, and the 6-7k peaks of the calibration clip get about the 1.5x it needed over 2-4k
	float g1 = settle(0.1f), g2 = settle(0.2f), g3 = settle(0.5f);
	assert(g1 > 1.0f && g2 > g1 && g3 >= g2);
	assert(g2 > 1.35f && g2 < 1.7f);
	// capped at WAVE_EXP_MAX for a full-scale hit
	assert(settle(1.0f) == WAVE_EXP_MAX);
	// the envelope rises at once and falls slowly: a hit keeps its gain for tens of ms, then lets go
	WaveExpand s = {};
	float hit = waveExpandGain(&s, 1.0f);
	assert(hit == WAVE_EXP_MAX);
	float after20 = 0;
	for (int i = 0; i < 80; i++)
		after20 = waveExpandGain(&s, 0.0f);
	assert(after20 > 1.5f);
	float after400 = 0;
	for (int i = 0; i < 1600; i++)
		after400 = waveExpandGain(&s, 0.0f);
	assert(after400 == 1.0f);
	puts("wave_expand ok");
	return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
    f = Path(td)/'test.cpp'
    f.write_text(src)
    subprocess.run(['g++', '-std=c++11', '-g', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', str(f), '-o', td+'/test'], check=True)
    subprocess.run([td+'/test'], check=True)
