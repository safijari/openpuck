import { describe, expect, it } from 'vitest';
import { directionOf, EMPTY_FILTER, parsePinSession, parseStream, passesFilter, type SnifferFrame } from '../sniffer';

const statusFrame = (o: { acq?: number; ch?: number; drops?: number; bondInfo?: number } = {}) => {
	const a = new Array(22).fill(0);
	a[0] = 0xc1;
	a[1] = 0xde;
	a[2] = o.acq ?? 1;
	a[3] = o.ch ?? 42;
	[a[4], a[5], a[6], a[7]] = [0xde, 0xad, 0xbe, 0xef];
	a[8] = 0x5a;
	a[18] = 3;
	a[19] = (o.drops ?? 0) & 0xff;
	a[20] = ((o.drops ?? 0) >> 8) & 0xff;
	a[21] = o.bondInfo ?? 0;
	return a;
};

const packetFrame = (op: number, payload: number[], o: { crc?: number; rssi?: number; ch?: number } = {}) => {
	const bytes = [payload.length + 2, 0x01, op, ...payload];
	return [
		0xc0,
		0xde,
		bytes.length,
		0x10,
		0x27,
		0x00,
		0x00, // 10000 us
		o.ch ?? 42,
		o.crc ?? 1,
		o.rssi ?? 60,
		1,
		...bytes,
	];
};

describe('parseStream', () => {
	it('decodes a status frame', () => {
		const { status, consumed } = parseStream(new Uint8Array(statusFrame({ ch: 7, drops: 300 })));
		expect(consumed).toBe(22);
		expect(status).toMatchObject({ capturing: true, channel: 7, drops: 300, prefix: 0x5a, lastPipe: 3 });
	});

	it('decodes the bond info nibbles', () => {
		// bit7 camped, bits 6-4 count, bits 3-0 channels
		const { status } = parseStream(new Uint8Array(statusFrame({ bondInfo: 0x80 | (2 << 4) | 5 })));
		expect(status).toMatchObject({ camped: true, bondCount: 2, bondChannels: 5 });
	});

	it('decodes a packet and its direction', () => {
		const { frames } = parseStream(new Uint8Array(packetFrame(0xe3, [])));
		expect(frames).toHaveLength(1);
		expect(frames[0]).toMatchObject({ dir: 'P→C', op: 'e3', crc: true, rssi: -60, tUs: 10000, ms: 10 });
	});

	it('keeps a large timestamp positive', () => {
		// A capture past ~35 minutes sets the top byte's high bit. The original
		// OR'd the multiplied byte in, and | coerces to int32, so the timestamp
		// went negative and the frame sorted before every earlier one.
		const f = packetFrame(0xf1, []);
		[f[3], f[4], f[5], f[6]] = [0x00, 0x00, 0x00, 0xff];
		const { frames } = parseStream(new Uint8Array(f));
		expect(frames[0].tUs).toBeGreaterThan(0);
		expect(frames[0].tUs).toBe(0xff * 16777216);
	});

	it('decodes several frames and a mixed stream', () => {
		const buf = new Uint8Array([...packetFrame(0xe3, []), ...statusFrame(), ...packetFrame(0xf1, [1, 2])]);
		const { frames, status } = parseStream(buf);
		expect(frames.map((f) => f.op)).toEqual(['e3', 'f1']);
		expect(status).not.toBeNull();
	});

	it('leaves a frame split across reads unconsumed', () => {
		const full = new Uint8Array(packetFrame(0xe3, [1, 2, 3]));
		const { frames, consumed } = parseStream(full.slice(0, full.length - 2));
		expect(frames).toHaveLength(0);
		expect(consumed).toBe(0);
	});

	it('resynchronises past leading garbage', () => {
		const buf = new Uint8Array([0x00, 0xff, 0x12, ...packetFrame(0xe3, [])]);
		expect(parseStream(buf).frames).toHaveLength(1);
	});

	it('resynchronises rather than trusting an impossible length', () => {
		const f = packetFrame(0xe3, []);
		f[2] = 200; // over MAX_FRAME
		expect(parseStream(new Uint8Array(f)).frames).toHaveLength(0);
	});

	it('marks a failed CRC', () => {
		const { frames } = parseStream(new Uint8Array(packetFrame(0xe3, [], { crc: 0 })));
		expect(frames[0].crc).toBe(false);
	});
});

describe('directionOf', () => {
	it('reads the opcode high nibble', () => {
		expect(directionOf(0xe3)).toBe('P→C');
		expect(directionOf(0xf1)).toBe('C→P');
		expect(directionOf(0x42)).toBe('?');
	});
});

describe('passesFilter', () => {
	const frame = (o: Partial<SnifferFrame>): SnifferFrame => ({
		seq: 1,
		tUs: 0,
		ms: 0,
		ch: 42,
		pipe: 1,
		dir: 'C→P',
		op: 'f1',
		crc: true,
		rssi: -60,
		len: 49,
		bytes: new Uint8Array([49, 1, 0xf1, 0xab, 0xcd]),
		...o,
	});

	it('passes everything with an empty filter', () => {
		expect(passesFilter(frame({}), EMPTY_FILTER)).toBe(true);
	});

	it('hides the two routine frames that make up most traffic', () => {
		const f = { ...EMPTY_FILTER, hideRoutine: true };
		expect(passesFilter(frame({ dir: 'C→P', op: 'f1', len: 49 }), f)).toBe(false);
		expect(passesFilter(frame({ dir: 'P→C', op: 'e3', len: 1 }), f)).toBe(false);
		// A same-opcode frame of a different length is not routine.
		expect(passesFilter(frame({ dir: 'C→P', op: 'f1', len: 20 }), f)).toBe(true);
	});

	it('filters by direction and opcode', () => {
		expect(passesFilter(frame({ dir: 'C→P' }), { ...EMPTY_FILTER, dir: 'P→C' })).toBe(false);
		expect(passesFilter(frame({ op: 'f1' }), { ...EMPTY_FILTER, op: 'E3' })).toBe(false);
		expect(passesFilter(frame({ op: 'f1' }), { ...EMPTY_FILTER, op: ' F1 ' })).toBe(true);
	});

	it('filters by length class', () => {
		expect(passesFilter(frame({ len: 49 }), { ...EMPTY_FILTER, len: 'ne49' })).toBe(false);
		expect(passesFilter(frame({ len: 50 }), { ...EMPTY_FILTER, len: 'gt49' })).toBe(true);
		expect(passesFilter(frame({ len: 1 }), { ...EMPTY_FILTER, len: 'gt1' })).toBe(false);
	});

	it('matches a hex substring ignoring spacing', () => {
		const e = frame({});
		expect(passesFilter(e, { ...EMPTY_FILTER, hexMatch: 'abcd' })).toBe(true);
		expect(passesFilter(e, { ...EMPTY_FILTER, hexMatch: 'ab cd' })).toBe(true);
		expect(passesFilter(e, { ...EMPTY_FILTER, hexMatch: 'dead' })).toBe(false);
	});
});

describe('parsePinSession', () => {
	it('accepts hex base/prefix with a decimal channel', () => {
		expect(parsePinSession('de ad be ef 5a 42')).toEqual([0xde, 0xad, 0xbe, 0xef, 0x5a, 42]);
	});

	it('accepts an explicit hex channel', () => {
		expect(parsePinSession('de ad be ef 5a 0x2a')).toEqual([0xde, 0xad, 0xbe, 0xef, 0x5a, 0x2a]);
	});

	it('rejects the wrong number of fields', () => {
		expect(parsePinSession('de ad be ef')).toBeNull();
		expect(parsePinSession('')).toBeNull();
	});

	it('rejects values that do not fit a byte', () => {
		expect(parsePinSession('de ad be ef 5a 300')).toBeNull();
		expect(parsePinSession('zz ad be ef 5a 42')).toBeNull();
	});
});
