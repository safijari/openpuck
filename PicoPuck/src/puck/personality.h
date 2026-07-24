// personality.h — the puck HID personality (feature command channel + status).
//
// Implements the TinyUSB HID get/set report callbacks (one instance per slot)
// and a periodic task that emits the 0x79 connection-state edges and the
// synthesized 0x43 battery report Steam reads. This is the port of OpenPuck's
// SteamPuckController.
//
// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef PICOPUCK_PERSONALITY_H
#define PICOPUCK_PERSONALITY_H

#include <stdbool.h>
#include <stdint.h>

void puck_personality_init(void);

// Periodic: connection-state (0x79) edges + battery (0x43). Call each loop.
void puck_personality_task(void);

// True while Steam is actively driving (has written to the command channel
// recently); used to decide whether we configure a controller ourselves.
bool puck_steam_active(void);

// Present a controller's input on `slot`. _synth serialises g_in[slot] into a
// limited report 0x45 (generic pads); _raw forwards an SC2's on-air report
// verbatim (rep[0] = report id). Both mark the slot live.
void puck_present_synth(int slot);
void puck_present_raw(int slot, const uint8_t *rep, uint8_t len);

// Mark a slot connected/disconnected (BT link up/down).
void puck_set_connected(int slot, bool connected);

#endif // PICOPUCK_PERSONALITY_H
