// OpenPuck channel-policy simulator.
//
// Simulates the puck<->controller link at the firmware's 250 Hz poll rate in modelled rooms, and compares:
//   OLD  : 0.9.59 -- fixed channel 18, no retry
//   CUR  : 0.9.67 -- one immediate retry + recovery on average delivery (<55% for 3 s), modelled from rf_link.cpp
//   NEW  : 0.9.67 retry + HitchPolicy (hitch_policy.cpp, the code that would ship)
// Player-felt metrics: hitches (gaps >= 40 ms between fresh reports), big freezes (>= 100 ms), worst freeze.
//
// Every room's parameters are randomised per seed so no policy is judged on one hand-picked layout.
#include "hitch_policy.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>

// ---------------------------------------------------------------- physics / room model
static const double SENS_DBM = -89.0; // nRF52840 2 Mbps sensitivity
static double CAPTURE_DB = 8.0; // SIR needed to survive a collision (GFSK co-channel rejection)
static const double POLL_MS = 4.0;
static const double RETRY_DELAY_MS = 1.5;

static double logistic(double x)
{
	return 1.0 / (1.0 + std::exp(-x));
}

// A transmitter that is on/off in bursts, optionally inside slower active/idle epochs (e.g. video chunks).
struct Interferer {
	double centerCh; // MHz above 2400
	double halfWidth; // MHz
	double powerDbm; // at the puck, inside its main lobe
	bool microwave = false;
	// burst process
	double burstMs, loadActive, loadIdle, epochActiveMs, epochIdleMs;
	// activity schedule: only on between [startMs, endMs) segments
	std::vector<std::pair<double, double>> on;
	// state
	std::mt19937_64 rng;
	bool epochActive = false, burstOn = false;
	double nextEpoch = 0, nextBurst = 0;

	bool scheduled(double t) const
	{
		if (on.empty())
			return true;
		for (auto &s : on)
			if (t >= s.first && t < s.second)
				return true;
		return false;
	}
	double attenuation(double ch) const
	{
		double d = std::fabs(ch - centerCh);
		if (microwave)
			return d <= 12 ? 0 : d <= 20 ? -15 : -40;
		if (d <= halfWidth - 1)
			return 0;
		if (d <= halfWidth + 1)
			return -10;
		if (d <= halfWidth + 10)
			return -28;
		return -40;
	}
	double expo(double mean)
	{
		std::exponential_distribution<double> e(1.0 / mean);
		return e(rng);
	}
	void advance(double t)
	{
		if (microwave)
			return;
		while (t >= nextEpoch) {
			epochActive = !epochActive;
			nextEpoch += expo(epochActive ? epochActiveMs : epochIdleMs);
		}
		while (t >= nextBurst) {
			double L = epochActive ? loadActive : loadIdle;
			burstOn = !burstOn;
			if (burstOn)
				nextBurst += expo(burstMs);
			else
				nextBurst += expo(burstMs * (1 - L) / L);
		}
	}
	bool active(double t)
	{
		if (!scheduled(t))
			return false;
		if (microwave)
			return std::fmod(t, 16.667) < 8.33; // magnetron fires on alternate mains half-cycles
		advance(t);
		return burstOn;
	}
};

struct Room {
	std::string name;
	double sigMean; // controller signal at the puck, dBm
	double slowSigma, slowTauMs; // body/movement fading
	double fastSigma; // per-packet fading
	double btLoss; // per-attempt loss from hopping neighbours (Bluetooth etc.)
	double chOffset[100]; // fixed per-channel multipath offset
	std::vector<Interferer> ifs;
};

static const double WIFI_CENTERS[3] = { 12, 37, 62 }; // Wi-Fi 1 / 6 / 11

static Interferer wifi(std::mt19937_64 &r, double center, double width,
		       double pwr, double burst, double la, double li,
		       double act, double idle)
{
	Interferer w;
	w.centerCh = center;
	w.halfWidth = width / 2;
	w.powerDbm = pwr;
	w.burstMs = burst;
	w.loadActive = la;
	w.loadIdle = li;
	w.epochActiveMs = act;
	w.epochIdleMs = idle;
	w.rng.seed(r());
	w.epochActive = std::uniform_real_distribution<double>(0, 1)(r) < 0.3;
	return w;
}

static double U(std::mt19937_64 &r, double a, double b)
{
	return std::uniform_real_distribution<double>(a, b)(r);
}
static int I(std::mt19937_64 &r, int a, int b)
{
	return std::uniform_int_distribution<int>(a, b)(r);
}

static Interferer lightNet(std::mt19937_64 &r, double pwr)
{
	return wifi(r, WIFI_CENTERS[I(r, 0, 2)], 20, pwr, U(r, 0.8, 2),
		    U(r, 0.05, 0.15), U(r, 0.01, 0.03), U(r, 2000, 5000),
		    U(r, 10000, 40000));
}
static Interferer streamNet(std::mt19937_64 &r, double center, double pwr)
{
	return wifi(r, center, 20, pwr, U(r, 2.5, 4.5), U(r, 0.35, 0.6),
		    U(r, 0.02, 0.05), U(r, 1500, 4000), U(r, 3000, 9000));
}

static Room makeRoom(int kind, uint64_t seed, double sessionMs)
{
	std::mt19937_64 r(seed * 7919 + kind);
	Room m;
	m.slowSigma = 3;
	m.slowTauMs = 2000;
	m.fastSigma = 3;
	m.btLoss = 0.003;
	std::normal_distribution<double> mp(0, 2.5);
	for (int c = 0; c < 100; c++)
		m.chOffset[c] = mp(r);
	switch (kind) {
	case 0:
		m.name = "Quiet house";
		m.sigMean = U(r, -60, -50);
		m.ifs.push_back(lightNet(r, U(r, -55, -45)));
		for (int i = I(r, 1, 2); i > 0; i--)
			m.ifs.push_back(lightNet(r, U(r, -85, -70)));
		break;
	case 1:
		m.name = "Home with someone streaming";
		m.sigMean = U(r, -62, -52);
		m.ifs.push_back(streamNet(r, WIFI_CENTERS[I(r, 0, 2)],
					  U(r, -52, -42)));
		for (int i = 2; i > 0; i--)
			m.ifs.push_back(lightNet(r, U(r, -80, -65)));
		break;
	case 2:
		m.name = "Crowded apartment";
		m.sigMean = U(r, -64, -54);
		m.btLoss = 0.01;
		for (int i = I(r, 6, 10); i > 0; i--) {
			double p = U(r, -80, -50);
			if (U(r, 0, 1) < 0.35)
				m.ifs.push_back(streamNet(
					r, WIFI_CENTERS[I(r, 0, 2)], p));
			else
				m.ifs.push_back(lightNet(r, p));
		}
		if (U(r, 0, 1) < 0.5) // a 40 MHz network
			m.ifs.push_back(wifi(r, U(r, 22, 52), 40,
					     U(r, -70, -55), 3, 0.3, 0.03, 3000,
					     6000));
		break;
	case 3:
		m.name = "Every channel busy";
		m.sigMean = U(r, -64, -54);
		m.btLoss = 0.01;
		for (int k = 0; k < 3; k++)
			for (int i = 2; i > 0; i--)
				m.ifs.push_back(streamNet(r, WIFI_CENTERS[k],
							  U(r, -65, -48)));
		m.ifs.push_back(wifi(r, U(r, 22, 52), 40, U(r, -65, -55), 3,
				     0.35, 0.05, 3000, 5000));
		for (int i = 4; i > 0; i--)
			m.ifs.push_back(lightNet(r, U(r, -80, -60)));
		break;
	case 4: {
		m.name = "Quiet house + microwave";
		m.sigMean = U(r, -60, -50);
		m.ifs.push_back(lightNet(r, U(r, -55, -45)));
		Interferer mw;
		mw.microwave = true;
		mw.centerCh = U(r, 52, 62);
		mw.halfWidth = 12;
		mw.powerDbm = U(r, -45, -30);
		double t = U(r, 2, 6) * 60000;
		for (int i = 0; i < 3 && t < sessionMs; i++) {
			double len = U(r, 1.5, 4) * 60000;
			mw.on.push_back({ t, t + len });
			t += len + U(r, 4, 9) * 60000;
		}
		m.ifs.push_back(mw);
		break;
	}
	case 5:
		m.name = "Weak signal (far/blocked)";
		m.sigMean = U(r, -86, -82);
		m.slowSigma = 4;
		m.slowTauMs = 1500;
		m.fastSigma = 4;
		m.ifs.push_back(lightNet(r, U(r, -60, -48)));
		m.ifs.push_back(lightNet(r, U(r, -80, -65)));
		break;
	case 6: {
		m.name = "Neighbor starts streaming";
		m.sigMean = U(r, -62, -52);
		m.ifs.push_back(lightNet(r, U(r, -60, -48)));
		Interferer a = streamNet(r, WIFI_CENTERS[I(r, 0, 2)],
					 U(r, -50, -40));
		a.on.push_back({ 10 * 60000.0, sessionMs });
		m.ifs.push_back(a);
		Interferer b = streamNet(r, WIFI_CENTERS[I(r, 0, 2)],
					 U(r, -55, -45));
		b.on.push_back({ 20 * 60000.0, sessionMs });
		m.ifs.push_back(b);
		break;
	}
	case 7:
		m.name = "Weak signal + crowded";
		m.sigMean = U(r, -84, -80);
		m.slowSigma = 4;
		m.slowTauMs = 1500;
		m.fastSigma = 4;
		m.btLoss = 0.01;
		for (int i = I(r, 6, 10); i > 0; i--) {
			double p = U(r, -80, -50);
			if (U(r, 0, 1) < 0.35)
				m.ifs.push_back(streamNet(
					r, WIFI_CENTERS[I(r, 0, 2)], p));
			else
				m.ifs.push_back(lightNet(r, p));
		}
		break;
	case 8: {
		m.name = "Bursty device (camera/backup)";
		m.sigMean = U(r, -62, -52);
		m.ifs.push_back(lightNet(r, U(r, -60, -48)));
		// something nearby that sends heavy 60-180 ms bursts every few seconds: a Wi-Fi camera
		// uploading clips, a cloud backup, a wireless video sender
		double b = U(r, 60, 180);
		double L = b / (b + U(r, 2000, 8000));
		Interferer x = wifi(r, WIFI_CENTERS[I(r, 0, 2)], 20,
				    U(r, -50, -40), b, L, L, 1e9, 1e9);
		x.epochActive = true;
		m.ifs.push_back(x);
		break;
	}
	}
	return m;
}

struct Link {
	Room *room;
	std::mt19937_64 rng;
	double slowFade = 0, lastT = 0;
	std::normal_distribution<double> n01{ 0, 1 };

	void advanceFade(double t)
	{
		double dt = t - lastT;
		lastT = t;
		double a = std::exp(-dt / room->slowTauMs);
		slowFade = a * slowFade +
			   std::sqrt(1 - a * a) * room->slowSigma * n01(rng);
	}
	// one transmission attempt on channel ch at time t; returns success and the reply's RSSI
	bool attempt(double t, int ch, double *rssi)
	{
		double s = room->sigMean + room->chOffset[ch] + slowFade +
			   room->fastSigma * n01(rng);
		double p = logistic((s - SENS_DBM) / 1.5);
		for (auto &f : room->ifs) {
			if (!f.active(t))
				continue;
			double sir = s - (f.powerDbm + f.attenuation(ch));
			p *= logistic((sir - CAPTURE_DB) / 2.0);
		}
		p *= 1 - room->btLoss;
		*rssi = s;
		return std::uniform_real_distribution<double>(0, 1)(rng) < p;
	}
};

// ---------------------------------------------------------------- 0.9.67 recovery model (from rf_link.cpp)
struct CurPolicy {
	static const int N = 39;
	static constexpr uint8_t POOL[14] = { 18, 20, 22, 34, 42, 46, 52,
					      56, 68, 70, 72, 74, 76, 80 };
	int ch = 18;
	double arrived = 0;
	int badStreak = 0;
	bool decided = false;
	double cooldownUntil = 0;
	uint64_t failed = 0;
	int hopIdx = 0;
	// evidence (all 39 channels)
	int evCount[N] = {};
	double evMean[N] = {}, evLast[N] = {};
	// history (pool)
	int hTrials[14] = {};
	double hWorst[14] = {};
	bool hGood[14] = {};
	int hGoodRun = 0, hBadRun = 0;
	bool hRecorded = false;

	static int idx(int c)
	{
		return (c - 4) / 2;
	}
	static int poolIdx(int c)
	{
		for (int i = 0; i < 14; i++)
			if (POOL[i] == c)
				return i;
		return -1;
	}
	void residenceChanged()
	{
		failed = 0;
		cooldownUntil = 0;
		decided = false;
		badStreak = 0;
		hGoodRun = hBadRun = 0;
		hRecorded = false;
	}
	void record(bool good, double worst)
	{
		int pi = poolIdx(ch);
		if (pi < 0 || hRecorded)
			return;
		hRecorded = true;
		if (!hTrials[pi])
			hWorst[pi] = worst;
		else
			hWorst[pi] = good ? (hWorst[pi] * 3 + worst) / 4 :
					    (hWorst[pi] + worst) / 2;
		hTrials[pi]++;
		hGood[pi] = good && hWorst[pi] >= 800;
	}
	// one 1-second window; returns target channel or 0
	int window(double now, int polls, int replies)
	{
		if (polls < 150)
			return 0;
		double succ = replies * 1000.0 / polls;
		int i = idx(ch);
		// evidence
		if (succ < 800) {
			evCount[i] = 0;
			evMean[i] = 0;
			evLast[i] = 0;
		} else {
			if (evCount[i] < 5) {
				evMean[i] = (evMean[i] * evCount[i] + succ) /
					    (evCount[i] + 1);
				evCount[i]++;
			} else
				evMean[i] = (evMean[i] * 4 + succ) / 5;
			evLast[i] = now;
		}
		// history
		if (succ >= 800) {
			hBadRun = 0;
			if (++hGoodRun >= 5)
				record(true, succ);
		} else {
			hGoodRun = 0;
			if (succ < 550) {
				if (++hBadRun >= 3)
					record(false, succ);
			} else
				hBadRun = 0;
		}
		bool cool = cooldownUntil > now;
		if (cool)
			badStreak = 0;
		else if (succ < 550)
			badStreak++;
		else
			badStreak = 0;
		if (cool || badStreak < 3 || now - arrived < 10000 || decided)
			return 0;
		decided = true;
		return select(now, succ);
	}
	bool isFailed(int c)
	{
		return failed & (1ull << idx(c));
	}
	int select(double now, double curWorst)
	{
		int best = 0;
		double bestMean = 0;
		for (int c = 4; c <= 80; c += 2) {
			if (c == ch || isFailed(c))
				continue;
			int i = idx(c);
			if (evCount[i] < 5 || evMean[i] < 800 ||
			    now - evLast[i] > 120000)
				continue;
			if (evMean[i] < curWorst + 200)
				continue;
			if (!best || evMean[i] > bestMean) {
				best = c;
				bestMean = evMean[i];
			}
		}
		if (best)
			return best;
		double bw = -1;
		for (int i = 0; i < 14; i++)
			if (POOL[i] != ch && !isFailed(POOL[i]) && hTrials[i] &&
			    hGood[i] && hWorst[i] > bw) {
				bw = hWorst[i];
				best = POOL[i];
			}
		if (best)
			return best;
		for (int s = 0; s < 14; s++) {
			int i = (hopIdx + s) % 14;
			if (POOL[i] == ch || isFailed(POOL[i]) || hTrials[i])
				continue;
			hopIdx = (i + 1) % 14;
			return POOL[i];
		}
		bw = -1;
		for (int i = 0; i < 14; i++)
			if (POOL[i] != ch && !isFailed(POOL[i]) && hTrials[i] &&
			    hWorst[i] >= curWorst + 200 && hWorst[i] > bw) {
				bw = hWorst[i];
				best = POOL[i];
			}
		return best;
	}
	void hopped(int c, double now, bool ok)
	{
		if (!ok) {
			failed |= 1ull << idx(c);
			cooldownUntil = now + 30000;
			decided = false;
			badStreak = 0;
			return;
		}
		ch = c;
		arrived = now;
		residenceChanged();
	}
};
constexpr uint8_t CurPolicy::POOL[14];

static bool g_randomStart = false;
// ---------------------------------------------------------------- session
enum Pol { OLD = 0, CUR = 1, NEW = 2, HYB = 3 };
static const char *POLNAME[4] = { "0.9.59 (old)", "0.9.67 (current)",
				  "Hitch-aware (new)",
				  "Hitch-aware + 0.9.67 backstop (firmware)" };

struct Result {
	double hitchPerMin, bigPerMin, maxGapMs, switches, delivered,
		futileEnd;
};

// Hop mechanics shared by CUR and NEW (the firmware's neutral-gated E4 handoff): HOP_GAP_MS without input
// while both ends retune, then the target is watched for 1.2 s and rolled back if it can't carry the link.
static double HOP_GAP_MS = 60;
static const double HOP_OBSERVE_MS = 1200;
static const double HOP_MIN_SUCCESS = 0.30;

static Result runSession(int roomKind, uint64_t seed, Pol pol,
			 double sessionMs)
{
	Room room = makeRoom(roomKind, seed, sessionMs);
	Link link;
	link.room = &room;
	link.rng.seed(seed * 104729 + 17);

	int ch = 18;
	if (g_randomStart) {
		std::mt19937_64 sr(seed * 31 + roomKind);
		ch = 4 + 2 * (int)(sr() % 39);
	}
	CurPolicy cur;
	cur.ch = ch;
	HitchPolicy hp;
	hp.begin((uint8_t)ch, 0);
	hp.setBackstop(pol != HYB);
	const bool useCur = pol == CUR || pol == HYB;
	const bool useHp = pol == NEW || pol == HYB;
	int hopBy = 0; // 1 = 0.9.67 recovery, 2 = hitch policy

	double lastReply = -1, maxGap = 0;
	long hitches = 0, big = 0, polls = 0, replies = 0, switches = 0;
	int winPolls = 0, winReplies = 0;
	double nextWin = 1000, nextTick = 100;

	// hop state
	double hopUntil = -1, observeUntil = -1;
	int hopFrom = 0, hopTo = 0, obsPolls = 0, obsReplies = 0;
	bool inRollback = false;

	static const bool dbg = getenv("SIMDBG") != nullptr;
	auto startHop = [&](int to, double t, int by) {
		hopBy = by;
		if (pol == HYB)
			hp.suspend();
		if (dbg)
			fprintf(stderr, "  [%s seed%llu] t=%.1fs hop %d->%d recentHitches=%u weak=%d\n",
				POLNAME[pol], (unsigned long long)seed, t / 1000, ch, to,
				useHp ? hp.recentHitches() : 0, useHp ? hp.weakSignal() : 0);
		hopFrom = ch;
		hopTo = to;
		hopUntil = t + HOP_GAP_MS;
		inRollback = false;
		switches++;
	};

	for (double t = 0; t < sessionMs; t += POLL_MS) {
		link.advanceFade(t);
		bool ok = false;
		double rssi = 0;
		if (hopUntil >= 0 && t < hopUntil) {
			ok = false; // retuning
		} else {
			if (hopUntil >= 0) { // hop gap just ended
				hopUntil = -1;
				ch = inRollback ? hopFrom : hopTo;
				if (!inRollback) {
					observeUntil = t + HOP_OBSERVE_MS;
					obsPolls = obsReplies = 0;
				}
			}
			ok = link.attempt(t, ch, &rssi);
			if (!ok && pol != OLD && lastReply >= 0 &&
			    t - lastReply < 300)
				ok = link.attempt(t + RETRY_DELAY_MS, ch, &rssi);
		}
		polls++;
		winPolls++;
		if (ok) {
			replies++;
			winReplies++;
			if (lastReply >= 0) {
				double g = t - lastReply;
				if (g >= 40)
					hitches++;
				if (g >= 100)
					big++;
				if (g > maxGap)
					maxGap = g;
			}
			lastReply = t;
		}
		const bool hopBusyNow = hopUntil >= 0 || observeUntil >= 0;
		// the firmware doesn't feed the policy while a move is in flight
		if (pol == NEW || (pol == HYB && !hopBusyNow))
			hp.notePoll((uint32_t)t, ok, (int8_t)(ok ? rssi : 0));

		// target acquisition after a hop. FIRMWARE rule (rf_link.cpp RF_GROUP_TARGET_ACQUIRE): the first fresh
		// reply within HOP_OBSERVE_MS completes the move; none -> roll back. SIM30 restores the stricter
		// "30% delivery over the window" rule the first runs used.
		if (observeUntil >= 0) {
			obsPolls++;
			obsReplies += ok;
			const bool strict = getenv("SIM30") != nullptr;
			bool done = strict ? t >= observeUntil :
					     (ok || t >= observeUntil);
			if (done) {
				observeUntil = -1;
				bool good = strict ? obsReplies >= HOP_MIN_SUCCESS *
									 obsPolls :
						     obsReplies > 0;
				if (good) {
					if (useCur)
						cur.hopped(hopTo, t, true);
					if (useHp) {
						if (pol == HYB && hopBy == 1)
							hp.externalMove(hopTo,
									(uint32_t)t);
						else
							hp.hopped(hopTo, (uint32_t)t,
								  true);
					}
				} else {
					if (dbg)
						fprintf(stderr, "  [%s seed%llu] t=%.1fs ROLLBACK from %d (%d/%d)\n",
							POLNAME[pol], (unsigned long long)seed, t / 1000, hopTo, obsReplies, obsPolls);
					// roll back to the previous channel
					hopUntil = t + HOP_GAP_MS;
					inRollback = true;
					if (useCur)
						cur.hopped(hopTo, t, false);
					if (useHp && (pol == NEW || hopBy == 2))
						hp.hopped(hopTo, (uint32_t)t, false);
				}
			}
		}
		bool hopBusy = hopUntil >= 0 || observeUntil >= 0;

		if (t >= nextWin) {
			nextWin += 1000;
			if (useCur && !hopBusy) {
				int target = cur.window(t, winPolls, winReplies);
				if (target)
					startHop(target, t, 1);
			}
			winPolls = winReplies = 0;
		}
		if (useHp && t >= nextTick) {
			nextTick += 100;
			hopBusy = hopUntil >= 0 || observeUntil >= 0;
			uint8_t target = hp.tick((uint32_t)t, !hopBusy);
			if (target && !hopBusy)
				startHop(target, t, 2);
		}
	}
	// a freeze still running at the end counts too
	if (lastReply >= 0 && sessionMs - lastReply > maxGap)
		maxGap = sessionMs - lastReply;
	double min = sessionMs / 60000.0;
	if (dbg)
		fprintf(stderr, "[%s seed%llu] room=%s hitches=%ld big=%ld maxgap=%.0f switches=%ld finalch=%d\n",
			POLNAME[pol], (unsigned long long)seed, room.name.c_str(), hitches, big, maxGap, switches, ch);
	return { hitches / min, big / min, maxGap, (double)switches,
		 replies / (sessionMs / 1000.0),
		 (double)(useHp ? hp.futileHops() : 0) };
}

int main(int argc, char **argv)
{
	if (getenv("CAPTURE_DB"))
		CAPTURE_DB = atof(getenv("CAPTURE_DB"));
	if (getenv("HOP_GAP_MS"))
		HOP_GAP_MS = atof(getenv("HOP_GAP_MS"));
	g_randomStart = getenv("RANDOM_START") != nullptr;
	int seeds = argc > 1 ? atoi(argv[1]) : 30;
	double minutes = argc > 2 ? atof(argv[2]) : 30;
	double sessionMs = minutes * 60000;
	printf("room,policy,hitches_per_min,big_per_min,worst_freeze_ms_median,worst_freeze_ms_p90,switches,reports_per_s\n");
	int only = argc > 3 ? atoi(argv[3]) : -1;
	for (int room = 0; room < 9; room++) {
		if (only >= 0 && room != only)
			continue;
		for (int p = 0; p < 4; p++) {
			std::vector<Result> rs;
			for (int s = 1; s <= seeds; s++)
				rs.push_back(runSession(room, (uint64_t)s,
							(Pol)p, sessionMs));
			double h = 0, b = 0, sw = 0, d = 0;
			std::vector<double> mg;
			for (auto &r : rs) {
				h += r.hitchPerMin;
				b += r.bigPerMin;
				sw += r.switches;
				d += r.delivered;
				mg.push_back(r.maxGapMs);
			}
			std::sort(mg.begin(), mg.end());
			int n = rs.size();
			std::string name =
				makeRoom(room, 1, sessionMs).name;
			printf("%s,%s,%.2f,%.3f,%.0f,%.0f,%.1f,%.0f\n",
			       name.c_str(), POLNAME[p], h / n, b / n,
			       mg[n / 2], mg[(int)(n * 0.9)], sw / n, d / n);
			fflush(stdout);
		}
	}
	return 0;
}
