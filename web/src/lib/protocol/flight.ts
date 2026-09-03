// Flight recorder: the events the firmware recorded in the seconds before its
// most recent watchdog hang. The ring lives in .noinit RAM so it survives the
// reset, making it the post-mortem of what wedged the board.
// From docs/index.html:1490-1556.

import { STAGE_NAMES } from './blob';

/** Stream the recorder: [0x10, restart?1:0], pumped until the end frame. */
export const FLIGHT_OP = 0x10;

/** Frame types inside a 0xA8 payload. */
export const FR_FRAME = { end: 0, entry: 1, header: 2 } as const;

/** Recorded event names, indexed by the firmware's event id. */
export const FR_EVT = [
	'none',
	'beat',
	'SET',
	'GET',
	'relay',
	'rf-up',
	'rf-DN',
	'HEAL!',
	'mount',
	'SUSPEND',
	'resume',
	'OFF',
	'RINGF',
	'save',
];

export const evtName = (e: number) => FR_EVT[e] ?? `evt${e}`;
export const stageName = (s: number) => STAGE_NAMES[s] ?? `st${s}`;

/** Vitals captured at the moment of the wedge. */
export interface FlightHeader {
	count: number;
	total: number;
	loopPerSec: number;
	stallMs: number;
	stage: number;
	usbdStk: number;
	loopStk: number;
	heap: number;
	pollsps: number;
	relayps: number;
	crc: number;
	norx: number;
	heal: number;
	ringF: number;
}

export interface FlightEvent {
	dt: number;
	evt: number;
	stage: number;
	arg: number;
}

/**
 * usbd stack free trending toward zero is the leading overflow suspect for the
 * multi-controller haptic-relay hang, so it is flagged inline.
 */
export const USBD_STACK_LOW = 16;

/**
 * Pull the header and events out of an accumulated buffer.
 *
 * Reports how much it consumed so a frame split across transfers is retried
 * rather than dropped -- the stream arrives in FIFO-sized batches.
 */
export function decodeFlightFrames(acc: Uint8Array): {
	header: FlightHeader | null;
	events: FlightEvent[];
	consumed: number;
	done: boolean;
} {
	let header: FlightHeader | null = null;
	const events: FlightEvent[] = [];
	let i = 0;
	let done = false;

	while (i < acc.length) {
		if (acc[i] !== 0xa8) {
			i++;
			continue;
		}
		if (i + 2 > acc.length) break;
		const L = acc[i + 1];
		if (i + 2 + L > acc.length) break;
		const f = acc.slice(i + 2, i + 2 + L);
		i += 2 + L;
		const T = f[0];

		if (T === FR_FRAME.end) {
			done = true;
			break;
		}
		if (T === FR_FRAME.header && L >= 27) {
			header = {
				count: f[1] | (f[2] << 8),
				total: f[3] | (f[4] << 8),
				loopPerSec: f[5] | (f[6] << 8),
				stallMs: f[7],
				stage: f[8],
				usbdStk: f[9] | (f[10] << 8),
				loopStk: f[11] | (f[12] << 8),
				heap: (f[13] | (f[14] << 8) | (f[15] << 16) | (f[16] << 24)) >>> 0,
				pollsps: f[17] | (f[18] << 8),
				relayps: f[19] | (f[20] << 8),
				crc: f[21],
				norx: f[22],
				heal: f[23] | (f[24] << 8),
				ringF: f[25] | (f[26] << 8),
			};
		} else if (T === FR_FRAME.entry && L >= 9) {
			events.push({
				dt: (f[1] | (f[2] << 8) | (f[3] << 16) | (f[4] << 24)) >>> 0,
				evt: f[5],
				stage: f[6],
				arg: f[7] | (f[8] << 8),
			});
		}
	}
	return { header, events, consumed: i, done };
}

const pad = (s: string | number, n: number) => String(s).padEnd(n);

/** The event trail as a fixed-column table, matching the console's FR output. */
export function formatFlight(events: FlightEvent[]): string {
	const hx = (v: number) => '0x' + v.toString(16).padStart(4, '0');
	return (
		pad('Δms', 8) +
		pad('event', 9) +
		pad('stage', 10) +
		'arg\n' +
		'─'.repeat(30) +
		'\n' +
		events.map((e) => pad(`-${e.dt}`, 8) + pad(evtName(e.evt), 9) + pad(stageName(e.stage), 10) + hx(e.arg)).join('\n')
	);
}

/** One-line summary of what the board looked like at the moment it wedged. */
export function formatWedgeVitals(h: FlightHeader): string {
	const low = h.usbdStk > 0 && h.usbdStk < USBD_STACK_LOW ? ' ⚠LOW' : '';
	return (
		`stuck in ${stageName(h.stage)}, stall ${h.stallMs}ms, loop ${h.loopPerSec}/s` +
		` · usbdStk ${h.usbdStk}w${low} · loopStk ${h.loopStk}w · heap ${h.heap}B` +
		` · poll ${h.pollsps} relay ${h.relayps} crc ${h.crc} norx ${h.norx} heal ${h.heal} ringF ${h.ringF}`
	);
}
