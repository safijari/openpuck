// RF sniffer: a separate board that listens to the puck <-> controller radio
// link and streams what it hears. From docs/sniffer.html.
//
// Two frame types share the stream, told apart by a 2-byte magic: 0xC1DE
// status (fixed 22 bytes) and 0xC0DE packet (variable).

export const SNIFFER_VID = 0x28de;

/** Frame magics: 0xC1DE is a fixed-size status, 0xC0DE a variable-length packet. */
export const MAGIC = { status: [0xc1, 0xde], packet: [0xc0, 0xde] } as const;

/** A status frame is always this long, magic included. */
export const STATUS_LEN = 22;
/** Frames run to MAXLEN+2; anything longer is a desync, not a packet. */
export const MAX_FRAME = 98;

/** Commands the sniffer board accepts on the OUT endpoint. */
export const SNIFFER_CMD = {
	reacquire: 0x01,
	pinChannel: 0x02,
	pinSession: 0x03,
	survey: 0x07,
	forgetBonds: 0x08,
	targetIbex: 0x09,
} as const;

export type Direction = 'P→C' | 'C→P' | '?';

export interface SnifferStatus {
	/** 1 = locked onto a session and capturing; otherwise scanning. */
	capturing: boolean;
	camped: boolean;
	channel: number;
	base: Uint8Array;
	prefix: number;
	lastPipe: number;
	advChannel: number;
	advBase: Uint8Array;
	advPrefix: number;
	/** Frames the device lost to a full ring. 0 means lossless. */
	drops: number;
	bondCount: number;
	bondChannels: number;
}

export interface SnifferFrame {
	seq: number;
	tUs: number;
	ms: number;
	ch: number;
	pipe: number;
	dir: Direction;
	/** Opcode as two hex digits. */
	op: string;
	crc: boolean;
	rssi: number;
	len: number;
	bytes: Uint8Array;
	/** Filled lazily; formatting every frame in the hot path stalls the read. */
	raw?: string;
	payload?: string;
}

export const hex = (b: ArrayLike<number>) => Array.from(b, (x: number) => x.toString(16).padStart(2, '0')).join(' ');

/** Direction comes from the opcode's high nibble. */
export function directionOf(op: number): Direction {
	if ((op & 0xf0) === 0xe0) return 'P→C';
	if ((op & 0xf0) === 0xf0) return 'C→P';
	return '?';
}

/** Decode a status frame: the locked session, what was advertised, and drop/bond counters. */
export function parseStatus(a: Uint8Array, i: number): SnifferStatus {
	const bondInfo = a[i + 21];
	return {
		capturing: a[i + 2] === 1,
		camped: !!(bondInfo & 0x80),
		channel: a[i + 3],
		base: a.slice(i + 4, i + 8),
		prefix: a[i + 8],
		lastPipe: a[i + 18],
		advChannel: a[i + 12],
		advBase: a.slice(i + 13, i + 17),
		advPrefix: a[i + 17],
		drops: a[i + 19] | (a[i + 20] << 8),
		bondCount: (bondInfo >> 4) & 0x7,
		bondChannels: bondInfo & 0x0f,
	};
}

export interface ParseResult {
	status: SnifferStatus | null;
	frames: Omit<SnifferFrame, 'seq'>[];
	consumed: number;
}

/**
 * Pull whole frames out of an accumulated buffer, resynchronising on garbage.
 * Returns how much was consumed so a frame split across reads is retried
 * rather than lost.
 */
export function parseStream(acc: Uint8Array): ParseResult {
	let status: SnifferStatus | null = null;
	const frames: Omit<SnifferFrame, 'seq'>[] = [];
	let i = 0;

	while (i + 2 <= acc.length) {
		if (acc[i] === MAGIC.status[0] && acc[i + 1] === MAGIC.status[1]) {
			if (i + STATUS_LEN > acc.length) break;
			status = parseStatus(acc, i);
			i += STATUS_LEN;
			continue;
		}
		if (acc[i] === MAGIC.packet[0] && acc[i + 1] === MAGIC.packet[1]) {
			if (i + 3 > acc.length) break;
			const n = acc[i + 2];
			if (n > MAX_FRAME) {
				i++; // bad length: resync rather than trust it
				continue;
			}
			if (i + 11 + n > acc.length) break;
			// The top byte is added, not OR'd in. The original multiplied it to
			// dodge the sign bit that << 24 would set, but then OR'd the result,
			// and | coerces both sides to int32 -- so any capture past ~35
			// minutes (2^31 us) still came out negative and sorted before every
			// earlier frame. Force the low three bytes unsigned, then add.
			const tUs = ((acc[i + 3] | (acc[i + 4] << 8) | (acc[i + 5] << 16)) >>> 0) + acc[i + 6] * 16777216;
			const bytes = acc.slice(i + 11, i + 11 + n);
			const op = bytes.length > 2 ? bytes[2] : 0;
			frames.push({
				tUs,
				ms: tUs / 1000,
				ch: acc[i + 7],
				pipe: acc[i + 10],
				dir: directionOf(op),
				op: op.toString(16).padStart(2, '0'),
				crc: !!(acc[i + 8] & 1),
				rssi: -acc[i + 9],
				len: bytes.length > 0 ? bytes[0] : 0,
				bytes,
			});
			i += 11 + n;
			continue;
		}
		i++; // resync
	}
	return { status, frames, consumed: i };
}

/** Fill in the hex strings, only for frames actually shown or exported. */
export function ensureHex(e: SnifferFrame): SnifferFrame {
	if (e.raw === undefined) {
		e.raw = hex(e.bytes);
		e.payload = hex(e.bytes.subarray(2));
	}
	return e;
}

export interface FilterSpec {
	hideRoutine: boolean;
	dir: string;
	op: string;
	len: string;
	hexMatch: string;
}

export const EMPTY_FILTER: FilterSpec = { hideRoutine: false, dir: '', op: '', len: '', hexMatch: '' };

/** Whether a frame survives the current filter row. */
export function passesFilter(e: SnifferFrame, f: FilterSpec): boolean {
	// The two frames that make up ~99% of traffic: input replies and bare polls.
	if (f.hideRoutine) {
		if (e.dir === 'C→P' && e.op === 'f1' && e.len === 49) return false;
		if (e.dir === 'P→C' && e.op === 'e3' && e.len <= 1) return false;
	}
	if (f.dir && e.dir !== f.dir) return false;
	const op = f.op.trim().toLowerCase();
	if (op && e.op !== op) return false;
	if (f.len === 'ne49' && e.len === 49) return false;
	if (f.len === 'gt49' && e.len <= 49) return false;
	if (f.len === 'gt1' && e.len <= 1) return false;
	const hx = f.hexMatch.trim().toLowerCase().replace(/ /g, '');
	if (hx && !ensureHex(e).payload!.replace(/ /g, '').includes(hx)) return false;
	return true;
}

/** "b0 b1 b2 b3 pfx ch" -- base and prefix hex, channel decimal or hex. */
export function parsePinSession(v: string): number[] | null {
	const parts = v
		.trim()
		.split(/[\s,]+/)
		.filter(Boolean);
	if (parts.length !== 6) return null;
	const nums = parts.map((p, i) => parseInt(p.replace(/^0x/i, ''), i === 5 && !/^0x/i.test(p) ? 10 : 16));
	if (nums.some((n) => !Number.isFinite(n) || n < 0 || n > 255)) return null;
	return nums;
}

/** Captured frames as plain text, one per line, for the .txt export. */
export const formatTextExport = (frames: SnifferFrame[]) =>
	frames
		.map(
			(e) =>
				`${e.ms.toFixed(1)}ms ch${e.ch} ${e.dir} ${e.crc ? '' : 'BAD '}rssi${e.rssi} op=${e.op} | ${ensureHex(e).raw}`,
		)
		.join('\n');
