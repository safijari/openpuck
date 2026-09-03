import { describe, expect, it } from 'vitest';
import { buildBlob, LIVE_V17 } from '../fixtures';
import { parseBlob } from '../blob';

describe('parseBlob', () => {
	const s = parseBlob(buildBlob());

	it('decodes the headline device state', () => {
		expect(s.protocol).toBe(17);
		expect(s.mode).toBe(5);
		expect(s.modeName).toBe('PS5 DualSense');
		expect(s.build).toEqual({ id: '0.9.31', dirty: false });
		expect(s.link).toEqual({ up: true, slot: 0, battery: 93, rssi: 43 });
	});

	it('decodes the link rates', () => {
		expect(s.rates.polls).toBe(LIVE_V17.polls);
		expect(s.rates.delivered).toBe(LIVE_V17.delivered);
		expect(s.rates.norx).toBe(4);
		expect(s.rates.relay).toBe(0);
	});

	it('reads the loop as running when the stall is under the 200ms threshold', () => {
		expect(s.loop.stalled).toBe(false);
		expect(s.loop.stage).toBe('rflink');
		expect(s.loop.worstStage).toBe('rflink');
		expect(s.loop.usbdStackWords).toBe(97);
		expect(s.loop.pollIntendedUs).toBe(4000);
	});

	it('flags a stall at or above 200ms', () => {
		const p = buildBlob();
		p[128] = 5; // 5 * 40ms = 200ms
		const stalled = parseBlob(p);
		expect(stalled.loop.stalled).toBe(true);
		expect(stalled.loop.stallMs).toBe(200);
	});

	it('treats xtal on both clocks as healthy', () => {
		expect(s.clock).toMatchObject({ lf: 'xtal', hf: 'xtal', usPerMs: 1000, lfBad: false, hfBad: false });
	});

	it('flags an RC-oscillator fallback and a drifting tick', () => {
		const p = buildBlob();
		p[122] = 1; // LF fell back to RC
		p[124] = 0xd0;
		p[125] = 0x03; // 976 us/ms
		const bad = parseBlob(p);
		expect(bad.clock).toMatchObject({ lf: 'RC', lfBad: true, usPerMsBad: true });
	});

	it('classifies a reboot as a non-fault reset', () => {
		expect(s.reset).toMatchObject({ code: 6, name: 'reboot', isFault: false, hangStageName: null });
	});

	it('classifies a watchdog hang as a fault and names the stuck stage', () => {
		const p = buildBlob();
		p[109] = 3; // watchdog (hang)
		p[126] = 5; // hung in haptic
		const crash = parseBlob(p);
		expect(crash.reset).toMatchObject({ name: 'watchdog (hang)', isFault: true, hangStageName: 'haptic' });
	});

	it('gates panel updates off below v15', () => {
		const p = buildBlob();
		p[0] = 14;
		expect(parseBlob(p).caps.panelUpdate).toBe(false);
		p[0] = 15;
		expect(parseBlob(p).caps.panelUpdate).toBe(true);
	});

	it('gates the post-v17 features off on a v17 puck', () => {
		expect(s.caps).toMatchObject({
			panelUpdate: true,
			lizard: true,
			typeConfig: true,
			slots: true,
			slotStats: true,
			dpadChords: false,
			gyroMap: false,
			padStick: false,
			rumble: false,
		});
	});

	it('falls back to firmware defaults for gated-off settings', () => {
		expect(s.chords.face).toEqual([3, 1, 4]);
		expect(s.chords.dpad).toEqual([9, 8, 7, 2]);
		expect(s.rumbleShaping.scalePct).toBe(200);
		expect(s.gyroLegacy).toBe(false);
	});

	it('reports only the bonded slot and its own counters', () => {
		expect(s.bondedCount).toBe(1);
		expect(s.slots[0]).toMatchObject({ up: true, battery: 93, rssi: 43 });
		expect(s.slots[0]?.stats).toMatchObject({ polls: 250, f1: 246, norx: 4 });
		expect(s.slots.slice(1)).toEqual([null, null, null]);
	});

	it('decodes signed IMU axes and their magnitude', () => {
		expect(s.imu).toEqual({ ax: 107, ay: -2371, az: 16323, magnitude: 16495 });
	});

	it('reads every per-type config block', () => {
		expect(s.types).toHaveLength(4);
		expect(s.types[0]).toMatchObject({ back: [5, 6, 7, 8], qam: 1, abSwap: false, padHaptics: true, rumble: true });
	});
});
