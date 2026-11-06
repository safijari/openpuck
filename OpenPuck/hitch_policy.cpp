#include "hitch_policy.h"
#include <math.h>
#include <string.h>

int HitchPolicy::idxOf(uint8_t ch)
{
	if (ch < HP_CH_MIN || ch > HP_CH_MAX || ((ch - HP_CH_MIN) & 1u))
		return -1;
	return (ch - HP_CH_MIN) / 2;
}

uint8_t HitchPolicy::chOf(int idx)
{
	return (uint8_t)(HP_CH_MIN + idx * 2);
}

void HitchPolicy::begin(uint8_t startCh, uint32_t now)
{
	memset(m_chs, 0, sizeof m_chs);
	m_ch = startCh;
	m_arrivedMs = m_secStartMs = m_lastDecayMs = now;
	m_haveReply = false;
	m_secPolls = m_secReplies = m_secHitches = 0;
	m_secRssiSum = 0;
	m_secRssiN = 0;
	memset(m_ringHitches, 0, sizeof m_ringHitches);
	memset(m_ringRssi, 0, sizeof m_ringRssi);
	m_ringPos = m_rssiPos = 0;
	m_weak = false;
	m_failStreak = 0;
	m_resSeconds = m_resHitches = 0;
	m_cooldownUntilMs = 0;
	m_futile = 0;
	m_judgePending = false;
	m_escapeUsed = false;
	m_rateBefore = 0;
	m_backstopDecided = false;
	m_rollbackUntilMs = 0;
}

void HitchPolicy::notePoll(uint32_t now, bool replied, int8_t rssiDbm)
{
	m_secPolls++;
	if (!replied)
		return;
	m_secReplies++;
	if (rssiDbm) {
		m_secRssiSum += rssiDbm;
		m_secRssiN++;
	}
	if (m_haveReply && now - m_lastReplyMs >= HP_HITCH_MS &&
	    m_secHitches < 0xFFFFu)
		m_secHitches++;
	m_lastReplyMs = now;
	m_haveReply = true;
}

void HitchPolicy::closeSecond()
{
	m_ringHitches[m_ringPos] =
		(uint8_t)(m_secHitches > 255u ? 255u : m_secHitches);
	m_ringPos = (uint8_t)((m_ringPos + 1u) % HP_RECENT_S);
	m_ringRssi[m_rssiPos] =
		m_secRssiN ? (int8_t)(m_secRssiSum / (int32_t)m_secRssiN) : 0;
	m_rssiPos = (uint8_t)((m_rssiPos + 1u) % HP_RSSI_S);
	{
		int32_t sum = 0;
		uint8_t n = 0;
		for (uint8_t i = 0; i < HP_RSSI_S; i++)
			if (m_ringRssi[i]) {
				sum += m_ringRssi[i];
				n++;
			}
		if (n >= 5u) {
			const int32_t avg = sum / n;
			if (avg < HP_WEAK_DBM)
				m_weak = true;
			else if (avg > HP_STRONG_DBM)
				m_weak = false;
		}
	}

	const int idx = idxOf(m_ch);
	// a second without polls (link down) says nothing about the channel
	if (idx >= 0 && m_secPolls) {
		HpChannel &c = m_chs[idx];
		if (c.seconds < 0xFFFFu)
			c.seconds++;
		c.hitches = (uint16_t)(c.hitches + m_secHitches > 0xFFFFu ?
					       0xFFFFu :
					       c.hitches + m_secHitches);
		if (m_resSeconds < 0xFFFFu)
			m_resSeconds++;
		m_resHitches =
			(uint16_t)(m_resHitches + m_secHitches > 0xFFFFu ?
					   0xFFFFu :
					   m_resHitches + m_secHitches);
	}
	if (m_secPolls >= 150u &&
	    (uint32_t)m_secReplies * 1000u <
		    (uint32_t)m_secPolls * HP_FAIL_PERMILLE) {
		if (m_failStreak < 255u)
			m_failStreak++;
	} else {
		m_failStreak = 0;
	}
	m_secPolls = m_secReplies = m_secHitches = 0;
	m_secRssiSum = 0;
	m_secRssiN = 0;
}

void HitchPolicy::decay(uint32_t now)
{
	if (now - m_lastDecayMs < HP_DECAY_MS)
		return;
	m_lastDecayMs = now;
	for (int i = 0; i < HP_CH_COUNT; i++) {
		m_chs[i].seconds /= 2u;
		m_chs[i].hitches /= 2u;
	}
}

uint32_t HitchPolicy::recentHitches() const
{
	uint32_t n = 0;
	for (uint8_t i = 0; i < HP_RECENT_S; i++)
		n += m_ringHitches[i];
	return n;
}

bool HitchPolicy::known(int idx) const
{
	return m_chs[idx].seconds >= 10u;
}

// Hitches arrive roughly at random, so a count h over T seconds only pins the rate down to h +/- ~sqrt(h).
// Moves compare the pessimistic bound of the destination with the optimistic bound of where we are.
float HitchPolicy::rateLo(int idx) const
{
	const HpChannel &c = m_chs[idx];
	if (!c.seconds)
		return 0;
	float h = (float)c.hitches - HP_Z * sqrtf((float)c.hitches);
	return h > 0 ? h * 60.0f / (float)c.seconds : 0;
}

float HitchPolicy::rateHi(int idx) const
{
	const HpChannel &c = m_chs[idx];
	if (!c.seconds)
		return 1e9f;
	const float h =
		(float)c.hitches + HP_Z * sqrtf((float)c.hitches + 1.0f) + 1.0f;
	return h * 60.0f / (float)c.seconds;
}

// Wi-Fi channels sit at fixed frequencies everywhere: 1 / 6 / 11 at 2412 / 2437 / 2462 MHz, plus 13 at 2472
// outside the US. When trying an unmeasured channel, the gaps between them are the better bet.
static int wifiCentreCloseness(int ch)
{
	static const int CENTRES[4] = { 12, 37, 62, 72 };
	int worst = 0;
	for (int k = 0; k < 4; k++) {
		const int d = ch > CENTRES[k] ? ch - CENTRES[k] :
						CENTRES[k] - ch;
		if (d < 10 && 10 - d > worst)
			worst = 10 - d;
	}
	return worst;
}

uint8_t HitchPolicy::pickTarget(float curLo, bool backstop, uint32_t now)
{
	const int cur = idxOf(m_ch);
	const float limit = curLo * HP_IMPROVE;
	// The backstop has no hitch rate to halve (the link is collapsing), so "better beyond doubt" there means
	// better than the current channel's own lower bound.
	const float knownLimit = backstop ? (cur >= 0 ? rateLo(cur) : 0) :
					    limit;

	// 1) a channel measured to be better beyond reasonable doubt
	int best = -1;
	float bestHi = 0;
	for (int i = 0; i < HP_CH_COUNT; i++) {
		if (i == cur || (int32_t)(m_chs[i].avoidUntilMs - now) > 0 ||
		    !known(i))
			continue;
		const float hi = rateHi(i);
		if (hi > knownLimit)
			continue;
		if (best < 0 || hi < bestHi) {
			best = i;
			bestHi = hi;
		}
	}
	if (best >= 0)
		return chOf(best);
	if (!backstop && curLo < HP_EXPLORE_MIN_PER_MIN)
		return 0;

	// 2) an unmeasured channel: not next to anything measured busy (a Wi-Fi network is ~20 MHz wide),
	// preferring the gaps between Wi-Fi channels, then the one furthest from known trouble
	for (int pass = 0; pass < 2; pass++) {
		best = -1;
		int bestClose = 0, bestClear = 0;
		for (int i = 0; i < HP_CH_COUNT; i++) {
			if (i == cur ||
			    (int32_t)(m_chs[i].avoidUntilMs - now) > 0 ||
			    known(i))
				continue;
			int clear = 255;
			bool nearBusy = false;
			for (int j = 0; j < HP_CH_COUNT; j++) {
				if (j != cur && !known(j))
					continue;
				const bool busy = j == cur || rateLo(j) > limit;
				if (!busy)
					continue;
				const int d =
					(i > j ? i - j : j - i) * 2; // MHz
				if (d < clear)
					clear = d;
				if (d <= 10)
					nearBusy = true;
			}
			// first pass keeps clear of known trouble; the backstop's second pass takes anything untried
			if (nearBusy && pass == 0)
				continue;
			const int close = wifiCentreCloseness(chOf(i));
			if (best < 0 || close < bestClose ||
			    (close == bestClose && clear > bestClear)) {
				best = i;
				bestClose = close;
				bestClear = clear;
			}
		}
		if (best >= 0 || !backstop)
			break;
	}
	if (best >= 0 || !backstop)
		return best < 0 ? 0 : chOf(best);

	// 3) backstop with everything tried: the best-measured channel
	for (int i = 0; i < HP_CH_COUNT; i++) {
		if (i == cur || (int32_t)(m_chs[i].avoidUntilMs - now) > 0 ||
		    !known(i))
			continue;
		const float hi = rateHi(i);
		if (best < 0 || hi < bestHi) {
			best = i;
			bestHi = hi;
		}
	}
	return best < 0 ? 0 : chOf(best);
}

void HitchPolicy::armCooldown()
{
	uint32_t cd = HP_BASE_COOLDOWN_MS << m_futile;
	if (cd > HP_MAX_COOLDOWN_MS)
		cd = HP_MAX_COOLDOWN_MS;
	m_cooldownUntilMs = m_arrivedMs + cd;
}

uint8_t HitchPolicy::tick(uint32_t now, bool canAct)
{
	while (now - m_secStartMs >= 1000u) {
		m_secStartMs += 1000u;
		closeSecond();
	}
	decay(now);

	const uint32_t resident = now - m_arrivedMs;
	const int cur = idxOf(m_ch);
	const float resRate = m_resSeconds ? (float)m_resHitches * 60.0f /
						     (float)m_resSeconds :
					     0;

	if (m_judgePending && resident >= HP_JUDGE_MS) {
		m_judgePending = false;
		if (resRate >= m_rateBefore * HP_FUTILE) {
			if (m_futile < 5u)
				m_futile++;
		} else {
			// one good move earns back one step of trust, so luck can't reset a long run of futile moves
			if (m_futile)
				m_futile--;
			m_escapeUsed = false;
		}
		armCooldown();
	}

	if (resident < HP_MIN_RESIDENCY_MS || !canAct)
		return 0;

	// Backstop: a collapsing link moves, on the same terms as the existing recovery -- once per residence, and not within the
	// retry cooldown of a move that had to be rolled back. If recent hitch-driven moves haven't helped, the
	// collapse is probably something that follows us everywhere, so it also waits out a capped back-off.
	if (m_backstopOn && m_failStreak >= HP_FAIL_WINDOWS &&
	    !m_backstopDecided && (int32_t)(m_rollbackUntilMs - now) <= 0) {
		const bool futileWait =
			!m_weak && m_futile &&
			(int32_t)(m_cooldownUntilMs - now) > 0 &&
			now - m_arrivedMs < HP_BACKSTOP_MAX_WAIT_MS;
		if (!futileWait) {
			m_backstopDecided = true;
			const uint8_t t = pickTarget(0, true, now);
			if (t) {
				m_rateBefore = resRate;
				m_failStreak = 0;
			}
			return t;
		}
	}

	if (recentHitches() < HP_TRIGGER_HITCHES)
		return 0;
	// Weak signal: hitches follow fading and range, which no channel fixes, and per-channel measurements are
	// mostly noise -- moving on them just ping-pongs. Leave it to the backstop above.
	if (weakSignal())
		return 0;

	// The last move made things worse: leave without sitting out the judging period -- once. A second bad
	// pick in a row waits like any other futile move, so bad luck can't turn into a hopping storm.
	const bool escape = m_judgePending && !m_escapeUsed &&
			    resRate > m_rateBefore;
	if (!escape &&
	    (m_judgePending || (int32_t)(m_cooldownUntilMs - now) > 0))
		return 0;

	const float curLo = cur >= 0 ? rateLo(cur) : 0;
	if (curLo <= 0)
		return 0; // not yet distinguishable from bad luck
	const uint8_t t = pickTarget(curLo, false, now);
	if (t) {
		if (escape) {
			m_escapeUsed = true;
			m_judgePending = false;
			if (m_futile < 5u)
				m_futile++;
		}
		m_rateBefore = resRate;
	}
	return t;
}

void HitchPolicy::hopped(uint8_t ch, uint32_t now, bool ok)
{
	m_haveReply =
		false; // the move's own pause is not the new channel's fault
	memset(m_ringHitches, 0, sizeof m_ringHitches);
	m_failStreak = 0;
	if (!ok) {
		const int idx = idxOf(ch);
		if (idx >= 0)
			m_chs[idx].avoidUntilMs = now + HP_FAILED_AVOID_MS;
		m_cooldownUntilMs = now + HP_BASE_COOLDOWN_MS;
		m_rollbackUntilMs = now + HP_BASE_COOLDOWN_MS;
		m_backstopDecided = false;
		return;
	}
	m_ch = ch;
	m_arrivedMs = now;
	m_resSeconds = m_resHitches = 0;
	m_judgePending = true;
	m_backstopDecided = false;
}

void HitchPolicy::externalMove(uint8_t ch, uint32_t now)
{
	m_haveReply = false;
	memset(m_ringHitches, 0, sizeof m_ringHitches);
	m_failStreak = 0;
	m_ch = ch;
	m_arrivedMs = now;
	m_resSeconds = m_resHitches = 0;
	// not our move: nothing to judge, and our own back-off stands
	m_judgePending = false;
	m_backstopDecided = false;
}
