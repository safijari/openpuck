import { describe, expect, it } from 'vitest';
import {
	BACKUP_MAGIC,
	bondRecord,
	bondedCount,
	buildBackup,
	hexDec,
	hexEnc,
	parseBackup,
	restoreSteps,
	type Backup,
} from '../backup';
import { buildBlob } from '../fixtures';
import { LZO, type LizardBinding } from '../lizard';

/** Bond dump: [count][mask][4 x 24-byte record]. */
function bondDump(usedSlots: number[]): Uint8Array {
	const bp = new Uint8Array(2 + 4 * 24);
	bp[0] = usedSlots.length;
	bp[1] = usedSlots.reduce((m, s) => m | (1 << s), 0);
	for (const s of usedSlots) {
		for (let i = 0; i < 24; i++) bp[2 + s * 24 + i] = s + 1;
	}
	return bp;
}

describe('hex helpers', () => {
	it('round-trips a bond record', () => {
		const bytes = Array.from({ length: 24 }, (_, i) => (i * 7) % 256);
		expect(hexDec(hexEnc(bytes))).toEqual(bytes);
	});

	it('pads single-digit bytes', () => {
		expect(hexEnc([0x0a, 0x00, 0xff])).toBe('0a00ff');
	});

	it('returns nothing for an empty record', () => {
		expect(hexDec('')).toEqual([]);
	});
});

describe('buildBackup', () => {
	const blob = buildBlob();

	it('captures the bond mask and records', () => {
		const b = buildBackup(blob, bondDump([0, 2]), null);
		expect(b.magic).toBe(BACKUP_MAGIC);
		expect(bondedCount(b)).toBe(2);
		expect(b.bonds[0].used).toBe(true);
		expect(b.bonds[1].used).toBe(false);
		expect(b.bonds[2].rec).toHaveLength(48); // 24 bytes hex-encoded
	});

	it('captures settings and every per-type block', () => {
		const b = buildBackup(blob, bondDump([0]), null);
		expect(b.config.mode).toBe(5);
		expect(b.config.chord).toEqual([3, 1, 4]);
		expect(b.config.types).toHaveLength(4);
		expect(b.config.types[0].back).toEqual([5, 6, 7, 8]);
	});

	it('omits settings this firmware predates rather than defaulting them', () => {
		// The fixture is v17: no D-pad chords, no gyro map, no pad-stick.
		// Writing defaults for these would silently change a newer target puck.
		const b = buildBackup(blob, bondDump([0]), null);
		expect(b.config).not.toHaveProperty('chordD');
		expect(b.config).not.toHaveProperty('swGyroLegacy');
		expect(b.config.types[0]).not.toHaveProperty('padStick');
	});

	it('includes the newer settings when the puck reports them', () => {
		const p = buildBlob();
		p[0] = 21;
		const wide = new Uint8Array(194);
		wide.set(p.slice(0, Math.min(p.length, 194)));
		wide[0] = 21;
		wide[180] = 9;
		wide[181] = 8;
		wide[182] = 7;
		wide[183] = 2;
		wide[184] = 1;
		const b = buildBackup(wide, bondDump([0]), null);
		expect(b.config.chordD).toEqual([9, 8, 7, 2]);
		expect(b.config.swGyroLegacy).toBe(1);
		expect(b.config.types[0].padStick).toHaveLength(2);
	});

	it('is version 1 without a lizard map and version 2 with one', () => {
		expect(buildBackup(blob, bondDump([0]), null).version).toBe(1);
		const map: LizardBinding[] = [{ outType: LZO.KBD, od: [1, 6, 0, 0, 0, 0, 0], trig: 0x1, hold: 0 }];
		const b = buildBackup(blob, bondDump([0]), map);
		expect(b.version).toBe(2);
		expect(b.config.lizardMap).toEqual(map);
	});

	it('copies the lizard map rather than aliasing it', () => {
		const map: LizardBinding[] = [{ outType: LZO.KBD, od: [1, 6, 0, 0, 0, 0, 0], trig: 0x1, hold: 0 }];
		const b = buildBackup(blob, bondDump([0]), map);
		map[0].od[0] = 0xff;
		expect(b.config.lizardMap![0].od[0]).toBe(1);
	});
});

describe('parseBackup', () => {
	const good = JSON.stringify({ magic: BACKUP_MAGIC, version: 1, bonds: [], config: {} });

	it('accepts a well-formed backup', () => {
		expect(parseBackup(good).magic).toBe(BACKUP_MAGIC);
	});

	it('rejects malformed JSON', () => {
		expect(() => parseBackup('{nope')).toThrow(/not valid JSON/);
	});

	it('rejects a JSON file that is not a backup', () => {
		expect(() => parseBackup('{"hello":1}')).toThrow(/unrecognized backup file/);
		expect(() => parseBackup(JSON.stringify({ magic: BACKUP_MAGIC }))).toThrow(/unrecognized backup file/);
	});
});

describe('bondRecord', () => {
	it('normalises a full record', () => {
		const rec = hexEnc(new Array(24).fill(0xab));
		expect(bondRecord({ slot: 0, used: true, rec })).toEqual({ used: 1, rec: new Array(24).fill(0xab) });
	});

	it('refuses a truncated record even when flagged used', () => {
		// A short record cannot be a valid bond; writing it would pair the target
		// puck to garbage.
		expect(bondRecord({ slot: 0, used: true, rec: 'abcd' }).used).toBe(0);
	});

	it('yields an empty slot for a missing bond', () => {
		expect(bondRecord(undefined)).toEqual({ used: 0, rec: new Array(24).fill(0) });
	});
});

describe('restoreSteps', () => {
	const base = { mode: 0, mDiv: 32, mFric: 40, persistMode: 0, chord: [3, 1, 4], types: [] };

	it('counts the minimum restore', () => {
		// 6 base fields + 4 bond slots + 1 commit
		expect(restoreSteps(base, false)).toBe(11);
	});

	it('adds the optional settings only when present', () => {
		expect(restoreSteps({ ...base, chordD: [1, 2, 3, 4] }, false)).toBe(15);
		expect(restoreSteps({ ...base, swGyroLegacy: 1 }, false)).toBe(12);
	});

	it('counts per-type blocks and their pad-stick pairs', () => {
		const types = [{ back: [0, 0, 0, 0], qam: 0, abSwap: 0, pad: 0, led: 0, padStick: [0, 0] }];
		expect(restoreSteps({ ...base, types }, false)).toBe(11 + 9 + 2);
	});

	it('skips the lizard map when the target puck cannot take one', () => {
		const c = { ...base, lizardMap: [{ outType: 1, od: [], trig: 0, hold: 0 }] };
		expect(restoreSteps(c, true)).toBe(12);
		expect(restoreSteps(c, false)).toBe(11);
	});
});
