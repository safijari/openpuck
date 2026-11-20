// link_stats.cpp -- see link_stats.h.
#include "link_stats.h"
#include "bonds.h"
#include "rf_link.h"
#include <string.h>

struct LsSlot {
	uint32_t lastReplyMs;
	bool haveReply;
	// the second being counted
	uint16_t secPolls, secReplies, secWorst;
	uint8_t secStut, secFrz;
	bool secConn;
	// the last LS_WINDOW_S closed seconds
	uint16_t rPolls[LS_WINDOW_S], rReplies[LS_WINDOW_S],
		rWorst[LS_WINDOW_S];
	uint8_t rStut[LS_WINDOW_S], rFrz[LS_WINDOW_S], rConn[LS_WINDOW_S];
	// since reset
	uint32_t totStut, totFrz, totConnSec;
	uint16_t totWorst;
};

struct LsMove {
	uint32_t ms;
	uint8_t from, to, cause;
	bool ok;
	uint16_t before,
		after; // stutters in the minute before / after (LS_AFTER_*)
	uint8_t beforeConn, afterConn; // connected seconds those counts cover
};
#define LS_AFTER_PENDING 0xFFFFu // the minute after hasn't passed yet
#define LS_AFTER_CUT 0xFFFEu // another move came first: no clean "after"

static LsSlot s_slot[NSLOT];
static uint8_t s_pos = 0; // next ring index to write
static uint32_t s_secStartMs = 0, s_resetMs = 0;
static bool s_started = false;
// all controllers together, for the move log's before/after
static uint8_t s_gStut[LS_WINDOW_S], s_gConn[LS_WINDOW_S];
static LsMove s_log[LS_LOG_MAX];
static uint8_t s_logN = 0, s_logHead = 0; // s_logHead = next write index

static uint16_t sat16(uint32_t v)
{
	return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v;
}

void linkStatsPoll(uint8_t slot, bool replied, uint32_t now)
{
	if (slot >= NSLOT)
		return;
	LsSlot &s = s_slot[slot];
	// only polls of a live link count toward delivery: cold/absent bonds are retried slowly and always miss
	const bool live = s.haveReply && now - s.lastReplyMs < LS_GAP_MAX_MS;
	if (live || replied)
		s.secPolls++;
	if (!replied)
		return;
	s.secReplies++;
	s.secConn = true;
	if (live) {
		const uint32_t gap = now - s.lastReplyMs;
		if (gap >= LS_STUTTER_MS) {
			if (s.secStut < 0xFF)
				s.secStut++;
			if (gap >= LS_FREEZE_MS && s.secFrz < 0xFF)
				s.secFrz++;
			if (gap > s.secWorst)
				s.secWorst = (uint16_t)gap;
		}
	}
	s.lastReplyMs = now;
	s.haveReply = true;
}

static uint16_t windowStutters(uint8_t *connOut)
{
	uint32_t st = 0;
	uint8_t conn = 0;
	for (unsigned i = 0; i < LS_WINDOW_S; i++) {
		st += s_gStut[i];
		conn += s_gConn[i] ? 1 : 0;
	}
	if (connOut)
		*connOut = conn;
	return sat16(st);
}

uint16_t linkStatsRecentStutters()
{
	return windowStutters(nullptr);
}

static void closeSecond()
{
	uint16_t gst = 0;
	bool gconn = false;
	for (int k = 0; k < NSLOT; k++) {
		LsSlot &s = s_slot[k];
		s.rPolls[s_pos] = s.secPolls;
		s.rReplies[s_pos] = s.secReplies;
		s.rWorst[s_pos] = s.secWorst;
		s.rStut[s_pos] = s.secStut;
		s.rFrz[s_pos] = s.secFrz;
		s.rConn[s_pos] = s.secConn ? 1 : 0;
		s.totStut += s.secStut;
		s.totFrz += s.secFrz;
		if (s.secConn)
			s.totConnSec++;
		if (s.secWorst > s.totWorst)
			s.totWorst = s.secWorst;
		gst += s.secStut;
		gconn |= s.secConn;
		s.secPolls = s.secReplies = s.secWorst = 0;
		s.secStut = s.secFrz = 0;
		s.secConn = false;
	}
	s_gStut[s_pos] = gst > 0xFF ? 0xFF : (uint8_t)gst;
	s_gConn[s_pos] = gconn ? 1 : 0;
	s_pos = (uint8_t)((s_pos + 1) % LS_WINDOW_S);
}

void linkStatsTick(uint32_t now)
{
	if (!s_started) {
		s_started = true;
		s_secStartMs = s_resetMs = now;
		return;
	}
	// catch up on missed seconds, but never spin: past a whole window everything is stale anyway
	for (unsigned n = 0; now - s_secStartMs >= 1000u; n++) {
		if (n < LS_WINDOW_S)
			closeSecond();
		s_secStartMs += 1000u;
	}
	// fill in "after" once a logged move has had its minute
	for (unsigned i = 0; i < s_logN; i++) {
		LsMove &m = s_log[i];
		if (m.after == LS_AFTER_PENDING &&
		    now - m.ms >= LS_WINDOW_S * 1000u)
			m.after = windowStutters(&m.afterConn);
	}
}

void linkStatsReset(uint32_t now)
{
	for (int k = 0; k < NSLOT; k++) {
		LsSlot &s = s_slot[k];
		const uint32_t keepMs = s.lastReplyMs;
		const bool keepHave = s.haveReply;
		memset(&s, 0, sizeof s);
		s.lastReplyMs = keepMs;
		s.haveReply = keepHave;
	}
	memset(s_gStut, 0, sizeof s_gStut);
	memset(s_gConn, 0, sizeof s_gConn);
	s_logN = s_logHead = 0;
	s_pos = 0;
	s_secStartMs = s_resetMs = now;
	s_started = true;
}

void linkStatsLogMove(uint8_t from, uint8_t to, uint8_t cause, bool ok,
		      uint32_t now)
{
	// an earlier move still waiting for its "after" minute is overtaken: this move changes the channel again
	for (unsigned i = 0; i < s_logN; i++)
		if (s_log[i].after == LS_AFTER_PENDING)
			s_log[i].after = LS_AFTER_CUT;
	LsMove &m = s_log[s_logHead];
	m.ms = now;
	m.from = from;
	m.to = to;
	m.cause = cause;
	m.ok = ok;
	m.before = windowStutters(&m.beforeConn);
	// a rolled-back move stays on the old channel, so there is no "after" for it to measure
	m.after = ok ? LS_AFTER_PENDING : LS_AFTER_CUT;
	m.afterConn = 0;
	s_logHead = (uint8_t)((s_logHead + 1) % LS_LOG_MAX);
	if (s_logN < LS_LOG_MAX)
		s_logN++;
}

static uint8_t *put16(uint8_t *q, uint16_t v)
{
	*q++ = (uint8_t)v;
	*q++ = (uint8_t)(v >> 8);
	return q;
}
static uint8_t *put32(uint8_t *q, uint32_t v)
{
	for (int i = 0; i < 4; i++)
		*q++ = (uint8_t)(v >> (8 * i));
	return q;
}

size_t linkStatsBuildFrame(uint8_t *out, size_t cap, uint32_t now,
			   const LinkStatsPolicy &pol)
{
	if (cap < LS_FRAME_LEN)
		return 0;
	uint8_t *q = out;
	*q++ = 1; // version
	*q++ = pol.channel;
	*q++ = pol.flags;
	*q++ = pol.futile;
	*q++ = pol.recentHitches;
	q = put32(q, (now - s_resetMs) / 1000u);
	for (int k = 0; k < NSLOT; k++) {
		const LsSlot &s = s_slot[k];
		uint32_t polls = 0, replies = 0, st = 0, fz = 0;
		uint16_t worst = 0;
		uint8_t conn = 0;
		for (unsigned i = 0; i < LS_WINDOW_S; i++) {
			polls += s.rPolls[i];
			replies += s.rReplies[i];
			st += s.rStut[i];
			fz += s.rFrz[i];
			conn += s.rConn[i];
			if (s.rWorst[i] > worst)
				worst = s.rWorst[i];
		}
		const bool up = g_slot[k].used && g_connReplyMs[k] &&
				now - g_connReplyMs[k] < 300u;
		*q++ = (uint8_t)((g_slot[k].used ? 1 : 0) | (up ? 2 : 0));
		*q++ = g_linkRssi[k];
		*q++ = conn;
		q = put16(q, sat16(st));
		q = put16(q, sat16(fz));
		q = put16(q, worst);
		q = put16(q, polls ? (uint16_t)((replies * 1000u) / polls) :
				     0xFFFFu);
		q = put32(q, s.totStut);
		q = put32(q, s.totFrz);
		q = put32(q, s.totConnSec);
		q = put16(q, s.totWorst);
	}
	*q++ = s_logN;
	// newest first
	for (unsigned i = 0; i < LS_LOG_MAX; i++) {
		if (i < s_logN) {
			const LsMove &m =
				s_log[(s_logHead + LS_LOG_MAX - 1 - i) %
				      LS_LOG_MAX];
			q = put16(q, sat16((now - m.ms) / 1000u));
			*q++ = m.from;
			*q++ = m.to;
			*q++ = (uint8_t)(m.cause | (m.ok ? 0x80 : 0));
			q = put16(q, m.before);
			*q++ = m.beforeConn;
			q = put16(q, m.after);
			*q++ = m.afterConn;
		} else {
			memset(q, 0, 11);
			q += 11;
		}
	}
	return (size_t)(q - out);
}
