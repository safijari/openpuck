// link_stats.h -- connection-quality numbers for the WebUSB panel: input stutters as the player feels them, and
// a log of the channel moves the puck made.
//
// A stutter is a gap of LS_STUTTER_MS or more between two fresh reports from the same controller (several
// 60 fps frames of stale input); a freeze is one of LS_FREEZE_MS or more. Gaps longer than LS_GAP_MAX_MS are a
// reconnect, not a stutter, and are left out. Counts are kept per controller over a rolling LS_WINDOW_S window
// and as totals since the last reset (boot or the panel's Reset button), along with how many seconds the
// controller was connected, so a rate per connected minute can be compared across boards and rooms.
//
// Loop context only (rf_link's poll path and hitchTask, and webusbPoll building the frame).
#pragma once
#include <stdint.h>
#include <stddef.h>

#define LS_STUTTER_MS 40u
#define LS_FREEZE_MS 100u
#define LS_GAP_MAX_MS 2500u
#define LS_WINDOW_S 60u
#define LS_LOG_MAX 8u

#define LS_CAUSE_HITCH 1 // the hitch policy moved the link (hitch_policy.h)
#define LS_CAUSE_OTHER \
	2 // the average-delivery recovery, a manual hop, or the journal builder

// One poll cycle for a slot: replied = a fresh report arrived.
void linkStatsPoll(uint8_t slot, bool replied, uint32_t now);
// Closes finished seconds; call at least once a second.
void linkStatsTick(uint32_t now);
// Zero every counter and the move log (the panel's Reset button).
void linkStatsReset(uint32_t now);
// A channel move finished. ok = landed on `to` (false = rolled back / aborted).
void linkStatsLogMove(uint8_t from, uint8_t to, uint8_t cause, bool ok,
		      uint32_t now);
// Stutters across all controllers in the last LS_WINDOW_S seconds.
uint16_t linkStatsRecentStutters();

// Policy state shown alongside the counters.
struct LinkStatsPolicy {
	uint8_t channel; // current session channel
	uint8_t flags; // LSP_*
	uint8_t futile; // hitch policy's futile-move streak
	uint8_t recentHitches; // hitch policy's last-20 s count
};
#define LSP_JUDGING \
	0x01 // the hitch policy is watching the link (exactly one controller connected)
#define LSP_WEAK 0x02 // weak signal: the policy leaves the channel alone
#define LSP_PENDING 0x04 // a move it asked for is in flight
#define LSP_AUTO 0x08 // automatic channel moves are enabled at all

// Payload of the 0xB5 frame (see webusb_config.cpp for the wire layout). Returns bytes written, 0 if cap is
// too small.
#define LS_FRAME_LEN (9u + 4u * 25u + 1u + LS_LOG_MAX * 11u)
size_t linkStatsBuildFrame(uint8_t *out, size_t cap, uint32_t now,
			   const LinkStatsPolicy &pol);
