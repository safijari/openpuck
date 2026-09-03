import { describe, expect, it } from 'vitest';
import { hexId, parseDongleStatus } from '../dongle';

function frame(flags: number, pucks: { slot: number; alive: boolean; serial: string }[]): Uint8Array {
	const p = new Uint8Array(3 + pucks.length * 26);
	p[0] = 0xac;
	p[1] = flags;
	p[2] = pucks.length;
	pucks.forEach((k, i) => {
		const q = 3 + i * 26;
		p[q] = k.slot;
		p[q + 1] = k.alive ? 1 : 0;
		for (let n = 0; n < 4; n++) p[q + 2 + n] = 0xa0 + n;
		for (let n = 0; n < 4; n++) p[q + 6 + n] = 0xb0 + n;
		// Serial field is fixed-width, NUL-padded.
		for (let n = 0; n < k.serial.length && n < 16; n++) p[q + 10 + n] = k.serial.charCodeAt(n);
	});
	return p;
}

describe('parseDongleStatus', () => {
	it('decodes the flag bits', () => {
		expect(parseDongleStatus(frame(0, []))).toMatchObject({ forwarding: false, linkUp: false });
		expect(parseDongleStatus(frame(1, []))).toMatchObject({ forwarding: true, linkUp: false });
		expect(parseDongleStatus(frame(2, []))).toMatchObject({ forwarding: false, linkUp: true });
		expect(parseDongleStatus(frame(3, []))).toMatchObject({ forwarding: true, linkUp: true });
	});

	it('decodes paired pucks with their ids', () => {
		const s = parseDongleStatus(frame(2, [{ slot: 1, alive: true, serial: 'ABC123' }]));
		expect(s.pucks).toHaveLength(1);
		expect(s.pucks[0]).toMatchObject({ slot: 1, alive: true, serial: 'ABC123' });
		expect(hexId(s.pucks[0].puuid)).toBe('a0a1a2a3');
		expect(hexId(s.pucks[0].iuuid)).toBe('b0b1b2b3');
	});

	it('strips NUL padding from a serial', () => {
		// The field is fixed-width, so an unpadded read shows trailing junk.
		expect(parseDongleStatus(frame(0, [{ slot: 0, alive: false, serial: 'AB' }])).pucks[0].serial).toBe('AB');
	});

	it('trims a serial at the first space', () => {
		expect(parseDongleStatus(frame(0, [{ slot: 0, alive: false, serial: 'AB CD' }])).pucks[0].serial).toBe('AB');
	});

	it('handles an empty pairing list', () => {
		expect(parseDongleStatus(frame(2, [])).pucks).toEqual([]);
	});

	it('drops a record the frame is too short to hold', () => {
		// Claiming 2 pucks but carrying 1 must not yield a half-decoded record.
		const p = frame(0, [{ slot: 0, alive: true, serial: 'A' }]);
		p[2] = 2;
		expect(parseDongleStatus(p).pucks).toHaveLength(1);
	});

	it('decodes several pucks', () => {
		const s = parseDongleStatus(
			frame(2, [
				{ slot: 0, alive: true, serial: 'ONE' },
				{ slot: 1, alive: false, serial: 'TWO' },
			]),
		);
		expect(s.pucks.map((k) => k.serial)).toEqual(['ONE', 'TWO']);
		expect(s.pucks.map((k) => k.alive)).toEqual([true, false]);
	});
});
