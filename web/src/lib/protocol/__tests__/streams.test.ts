import { describe, expect, it } from 'vitest';
import { CAP_SLOT, decodeCaptureFrames, formatCapture, formatEntry } from '../capture';
import { decodeFlightFrames, formatWedgeVitals, evtName, type FlightHeader } from '../flight';

/** Wrap a payload in the [mark][len] framing both streams share. */
const frame = (mark: number, payload: number[]) => [mark, payload.length, ...payload];
const u32 = (v: number) => [v & 0xff, (v >>> 8) & 0xff, (v >>> 16) & 0xff, (v >>> 24) & 0xff];

describe('capture stream', () => {
	const entry = (ms: number, slot: number, rid: number, bytes: number[]) =>
		frame(0xa6, [1, ...u32(ms), slot, rid, bytes.length, ...bytes]);

	it('decodes entries and reports the end frame', () => {
		const buf = new Uint8Array([...entry(100, 0, 0x40, [1, 2]), ...frame(0xa6, [0])]);
		const { entries, done } = decodeCaptureFrames(buf);
		expect(done).toBe(true);
		expect(entries).toHaveLength(1);
		expect(entries[0]).toMatchObject({ ms: 100, slot: 0, rid: 0x40, nb: 2, bytes: [1, 2] });
	});

	it('leaves a frame split across reads unconsumed', () => {
		// The ring is drained over many transfers; consuming a partial frame
		// would drop the entry it carries.
		const full = new Uint8Array(entry(100, 0, 0x40, [1, 2]));
		const { entries, consumed } = decodeCaptureFrames(full.slice(0, full.length - 3));
		expect(entries).toHaveLength(0);
		expect(consumed).toBe(0);
	});

	it('decodes several entries from one buffer', () => {
		const buf = new Uint8Array([...entry(10, 0, 0x40, [1]), ...entry(20, 0xfe, 0x87, [2, 3])]);
		expect(decodeCaptureFrames(buf).entries).toHaveLength(2);
	});

	it('times entries relative to the newest', () => {
		const e = { ms: 900, slot: 0, rid: 0x40, nb: 1, bytes: [0xab] };
		expect(formatEntry(e, 1000)).toContain('-   100ms');
	});

	it('labels each traffic direction distinctly', () => {
		const mk = (slot: number) => ({ ms: 0, slot, rid: 0x87, nb: 1, bytes: [1] });
		expect(formatEntry(mk(CAP_SLOT.txToController), 0)).toContain('TX→ctlr');
		expect(formatEntry(mk(CAP_SLOT.hostGet), 0)).toContain('GET←host');
		expect(formatEntry(mk(CAP_SLOT.toHost), 0)).toContain('→host');
		expect(formatEntry(mk(2), 0)).toContain('if2');
	});

	it('renders RF link edges as banners', () => {
		const mk = (k: number) => ({ ms: 0, slot: CAP_SLOT.linkEdge, rid: 0, nb: 1, bytes: [k] });
		expect(formatEntry(mk(1), 0)).toContain('RF LINK UP');
		expect(formatEntry(mk(0), 0)).toContain('RF LINK DOWN');
		expect(formatEntry(mk(2), 0)).toContain('RECONNECT');
	});

	it('formats an empty capture as empty', () => {
		expect(formatCapture([])).toBe('');
	});
});

describe('flight recorder stream', () => {
	const header = (o: Partial<FlightHeader> = {}) => {
		const h = {
			count: 3,
			total: 40,
			loopPerSec: 250,
			stallMs: 8000,
			stage: 5,
			usbdStk: 97,
			loopStk: 200,
			heap: 4096,
			pollsps: 250,
			relayps: 0,
			crc: 0,
			norx: 4,
			heal: 0,
			ringF: 0,
			...o,
		};
		return frame(0xa8, [
			2,
			h.count & 0xff,
			h.count >> 8,
			h.total & 0xff,
			h.total >> 8,
			h.loopPerSec & 0xff,
			h.loopPerSec >> 8,
			h.stallMs & 0xff,
			h.stage,
			h.usbdStk & 0xff,
			h.usbdStk >> 8,
			h.loopStk & 0xff,
			h.loopStk >> 8,
			...u32(h.heap),
			h.pollsps & 0xff,
			h.pollsps >> 8,
			h.relayps & 0xff,
			h.relayps >> 8,
			h.crc,
			h.norx,
			h.heal & 0xff,
			h.heal >> 8,
			h.ringF & 0xff,
			h.ringF >> 8,
		]);
	};
	const event = (dt: number, evt: number, stage: number, arg: number) =>
		frame(0xa8, [1, ...u32(dt), evt, stage, arg & 0xff, arg >> 8]);

	it('decodes the wedge header', () => {
		const { header: h } = decodeFlightFrames(new Uint8Array(header()));
		expect(h).toMatchObject({ total: 40, stage: 5, stallMs: 8000 & 0xff, usbdStk: 97, heap: 4096 });
	});

	it('decodes events and the end frame', () => {
		const buf = new Uint8Array([...header(), ...event(120, 4, 5, 0x1234), ...frame(0xa8, [0])]);
		const { events, done } = decodeFlightFrames(buf);
		expect(done).toBe(true);
		expect(events[0]).toEqual({ dt: 120, evt: 4, stage: 5, arg: 0x1234 });
	});

	it('names events and falls back for unknown ones', () => {
		expect(evtName(4)).toBe('relay');
		expect(evtName(6)).toBe('rf-DN');
		expect(evtName(99)).toBe('evt99');
	});

	it('flags a usbd stack low enough to suspect overflow', () => {
		const low = decodeFlightFrames(new Uint8Array(header({ usbdStk: 8 }))).header!;
		expect(formatWedgeVitals(low)).toContain('⚠LOW');
	});

	it('does not flag a healthy stack, or a zero reading', () => {
		const ok = decodeFlightFrames(new Uint8Array(header({ usbdStk: 97 }))).header!;
		expect(formatWedgeVitals(ok)).not.toContain('⚠LOW');
		// 0 means "not reported", not "critically low".
		const none = decodeFlightFrames(new Uint8Array(header({ usbdStk: 0 }))).header!;
		expect(formatWedgeVitals(none)).not.toContain('⚠LOW');
	});

	it('names the stage the board was stuck in', () => {
		const h = decodeFlightFrames(new Uint8Array(header({ stage: 5 }))).header!;
		expect(formatWedgeVitals(h)).toContain('stuck in haptic');
	});
});
