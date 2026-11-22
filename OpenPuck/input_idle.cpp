#include "input_idle.h"
#include "bonds.h"
#include <Arduino.h>

// Deliberate presses only. The capacitive stick/pad touch bits are left out: a thumb resting on a stick
// while waiting in a menu is not play.
#define IDLE_PRESS_MASK                                                        \
	(TB_A | TB_B | TB_X | TB_Y | TB_QAM | TB_R3 | TB_VIEW | TB_R4 |        \
	 TB_R5 | TB_RB | TB_DDN | TB_DRT | TB_DLF | TB_DUP | TB_MENU | TB_L3 | \
	 TB_STEAM | TB_L4 | TB_L5 | TB_LB | TB_RPADC | TB_LPADC | TB_R2 |      \
	 TB_L2)
// ~18% stick deflection: well past resting drift, well short of deliberate movement
#define IDLE_STICK 6000
#define IDLE_TRIGGER 25u

static unsigned long s_lastActiveMs[NSLOT];

static bool deflected(int16_t v)
{
	return v > IDLE_STICK || v < -IDLE_STICK;
}

void inputIdleNote(uint8_t slot, uint32_t rawButtons, const PuckInput &in)
{
	if (slot >= NSLOT)
		return;
	if ((rawButtons & IDLE_PRESS_MASK) || deflected(in.lx) ||
	    deflected(in.ly) || deflected(in.rx) || deflected(in.ry) ||
	    in.lt > IDLE_TRIGGER || in.rt > IDLE_TRIGGER)
		s_lastActiveMs[slot] = millis();
}

bool inputIdleQuiet()
{
	const unsigned long now = millis();
	for (int s = 0; s < NSLOT; s++) {
		// only controllers currently answering polls count
		if (!g_slot[s].used || !g_connReplyMs[s] ||
		    now - g_connReplyMs[s] >= 300u)
			continue;
		if (s_lastActiveMs[s] &&
		    now - s_lastActiveMs[s] < INPUT_IDLE_QUIET_MS)
			return false;
	}
	return true;
}
