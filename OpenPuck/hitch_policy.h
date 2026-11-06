// hitch_policy.h -- channel-switching policy driven by input hitches the player can feel.
//
// The existing recovery policy reacts to AVERAGE delivery (success < 55% for 3 s). Interference is bursty: a
// link that averages 85-90% can still freeze input for 50-200 ms every few seconds and never qualify. This
// policy counts hitches -- gaps between delivered reports long enough to notice -- and moves only when
// another channel is better beyond reasonable doubt. Nothing in it is fitted to a particular room:
//   - the hitch length is about human perception, not RF conditions;
//   - channels are judged against each other, with statistical bounds so chance doesn't trigger a move;
//   - it backs off when moving stops paying off (every channel busy);
//   - on a weak signal it leaves the channel alone (range problems aren't channel problems).
// Hardware-free: the caller feeds per-poll outcomes and acts on the returned target, so the same code runs in
// the firmware and in the host simulator.
#pragma once
#include <stdint.h>

#define HP_CH_MIN 4
#define HP_CH_MAX 80
#define HP_CH_COUNT \
	39 // HP_CH_MIN..HP_CH_MAX step 2, the set the firmware may recover onto

// A freeze this long is noticeable in play (several 60 fps frames of stale input).
#define HP_HITCH_MS 40u
// Recent-history span the current channel is judged on.
#define HP_RECENT_S 20u
// Hitches within HP_RECENT_S before a move is even considered: the problem must be felt, not just measured.
#define HP_TRIGGER_HITCHES 2u
#define HP_MIN_RESIDENCY_MS 10000u
// How long to stay after a move before judging whether it helped.
#define HP_JUDGE_MS 30000u
#define HP_BASE_COOLDOWN_MS 30000u
#define HP_MAX_COOLDOWN_MS 600000u
// A move must be expected to at least halve the hitch rate...
#define HP_IMPROVE 0.5f
// ...and counts as futile unless it actually got the rate under 70% of where it was.
#define HP_FUTILE 0.7f
// Trying a never-measured channel is only worth it once hitches are clearly frequent (per minute).
#define HP_EXPLORE_MIN_PER_MIN 1.0f
// Width of the "beyond reasonable doubt" bounds, in standard deviations of a Poisson count.
#define HP_Z 1.5f
// Within ~10 dB of the nRF52840's 2 Mbps sensitivity (-89 dBm) losses come from range, not the channel. Once
// weak, it takes a clearly stronger signal to count as strong again: averages taken only over replies that got
// through read high, and a link hovering at the edge mustn't flicker between modes.
#define HP_WEAK_DBM (-80)
#define HP_STRONG_DBM (-75)
#define HP_RSSI_S 60u // seconds of reply RSSI averaged
// Total-failure backstop, same terms as the existing recovery: delivery under 55% for 3 straight seconds.
#define HP_FAIL_PERMILLE 550u
#define HP_FAIL_WINDOWS 3u
// Longest the backstop defers to back-off before moving anyway.
#define HP_BACKSTOP_MAX_WAIT_MS 60000u
// Measurements fade so a channel that was busy earlier gets another chance once its history is stale.
#define HP_DECAY_MS 300000u
// A channel whose move failed outright is avoided for this long.
#define HP_FAILED_AVOID_MS 300000u

struct HpChannel {
	uint16_t seconds; // seconds of residence measured (decays)
	uint16_t hitches; // hitches during that residence (decays)
	uint32_t avoidUntilMs; // failed move -> skip until then
};

class HitchPolicy {
    public:
	void begin(uint8_t startCh, uint32_t now);
	// One poll cycle (after any retry): did a fresh report arrive, and its RSSI in dBm (0 if unknown).
	void notePoll(uint32_t now, bool replied, int8_t rssiDbm);
	// Call at least once per second. Returns the channel to move to, or 0 to stay. canAct = false keeps the
	// bookkeeping going while the caller couldn't start a move anyway (another move in flight, several
	// controllers live, ...).
	uint8_t tick(uint32_t now, bool canAct = true);
	// The caller completed (ok) or rolled back (!ok) a move this policy asked for.
	void hopped(uint8_t ch, uint32_t now, bool ok);
	// Something else moved the link (the average-delivery recovery, a manual hop, the journal builder).
	void externalMove(uint8_t ch, uint32_t now);
	// Polls were not fed for a while: the next reply must not count the gap as a hitch.
	void suspend()
	{
		m_haveReply = false;
	}
	// Off when the caller keeps its own total-failure recovery (the firmware does).
	void setBackstop(bool on)
	{
		m_backstopOn = on;
	}

	uint8_t channel() const
	{
		return m_ch;
	}
	uint32_t recentHitches() const;
	bool weakSignal() const
	{
		return m_weak;
	}
	uint8_t futileHops() const
	{
		return m_futile;
	}

    private:
	static int idxOf(uint8_t ch);
	static uint8_t chOf(int idx);
	bool known(int idx) const;
	// hitches per minute: plausible lower / upper bounds given how long the channel was watched
	float rateLo(int idx) const;
	float rateHi(int idx) const;
	uint8_t pickTarget(float curLo, bool backstop, uint32_t now);
	void closeSecond();
	void decay(uint32_t now);
	void armCooldown();

	HpChannel m_chs[HP_CH_COUNT];
	uint8_t m_ch = 18;
	uint32_t m_arrivedMs = 0, m_lastReplyMs = 0, m_secStartMs = 0;
	bool m_haveReply = false;

	// per-second accumulators and the recent ring
	uint16_t m_secPolls = 0, m_secReplies = 0, m_secHitches = 0;
	int32_t m_secRssiSum = 0;
	uint16_t m_secRssiN = 0;
	uint8_t m_ringHitches[HP_RECENT_S] = {};
	int8_t m_ringRssi[HP_RSSI_S] = {}; // 0 = no data that second
	uint8_t m_ringPos = 0, m_rssiPos = 0;
	bool m_weak = false;
	uint8_t m_failStreak = 0;

	// this residence only
	uint16_t m_resSeconds = 0, m_resHitches = 0;

	// move bookkeeping
	uint32_t m_cooldownUntilMs = 0;
	uint8_t m_futile = 0;
	bool m_judgePending = false;
	bool m_escapeUsed = false;
	bool m_backstopOn = true;
	bool m_backstopDecided =
		false; // backstop already moved (or tried) from this residence
	uint32_t m_rollbackUntilMs = 0;
	float m_rateBefore = 0; // per minute, on the channel we left
	uint32_t m_lastDecayMs = 0;
};
