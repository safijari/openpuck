// Host <-> controller command capture. The firmware logs everything from boot
// into a big RAM ring: Steam's writes (ifN), the frames we transmit to the
// controller (TX->ctlr), and RF link edges. From docs/index.html:1403-1453.
//
// Only present in a logging build (-DOPK_LOG=1), which the status blob reports
// as logEnabled.

export const CAP_OP = {
	/** [0x05, 1|0] arms or disarms the ring. */
	arm: 0x05,
	/** Drain whatever is buffered. */
	drain: 0x06,
} as const;

/** Frame types inside a 0xA6 payload. */
export const CAP_FRAME = { end: 0, entry: 1 } as const;

/**
 * Pseudo-slots the firmware uses for non-interface traffic. Real values 0..3
 * are Steam's HID interfaces.
 */
export const CAP_SLOT = {
	linkEdge: 0xfd,
	hostGet: 0xfc,
	toHost: 0xfb,
	txToController: 0xfe,
} as const;

export interface CaptureEntry {
	ms: number;
	slot: number;
	rid: number;
	nb: number;
	bytes: number[];
}

const hex2 = (v: number) => v.toString(16).padStart(2, '0');

/** One capture line, timestamped relative to the newest entry. */
export function formatEntry(e: CaptureEntry, maxMs: number): string {
	const hex = e.bytes.map(hex2).join(' ');
	const t = `-${String(maxMs - e.ms).padStart(6, ' ')}ms  `;

	if (e.slot === CAP_SLOT.linkEdge) {
		const k = e.bytes[0];
		const what = k === 2 ? 'RECONNECT (block+reinit armed)' : k === 1 ? 'RF LINK UP' : 'RF LINK DOWN';
		return `${t}${'═'.repeat(8)} ${what} ${'═'.repeat(8)}`;
	}
	// The host's battery poll arrives as a feature GET.
	if (e.slot === CAP_SLOT.hostGet) return `${t}GET←host rid=${hex2(e.rid)}  n=${e.nb}:  ${hex}`;
	// Pushed device->host status (0x43 battery / 0x7B / 0x79).
	if (e.slot === CAP_SLOT.toHost) return `${t}→host    rid=${hex2(e.rid)}  n=${e.nb}:  ${hex}`;

	const who = e.slot === CAP_SLOT.txToController ? 'TX→ctlr ' : `if${e.slot}     `;
	return `${t}${who} cmd=${hex2(e.rid)}  n=${e.nb}:  ${hex}`;
}

/** The whole capture as text, timestamped relative to the newest entry. */
export function formatCapture(lines: CaptureEntry[]): string {
	if (!lines.length) return '';
	const maxMs = lines[lines.length - 1].ms;
	return lines.map((e) => formatEntry(e, maxMs)).join('\n');
}

/** Decode capture entries out of an accumulated buffer of 0xA6 frames. */
export function decodeCaptureFrames(acc: Uint8Array): {
	entries: CaptureEntry[];
	consumed: number;
	done: boolean;
} {
	const entries: CaptureEntry[] = [];
	let i = 0;
	let done = false;
	while (i < acc.length) {
		if (acc[i] !== 0xa6) {
			i++;
			continue;
		}
		if (i + 2 > acc.length) break;
		const L = acc[i + 1];
		if (i + 2 + L > acc.length) break;
		const f = acc.slice(i + 2, i + 2 + L);
		i += 2 + L;
		if (f[0] === CAP_FRAME.end) {
			done = true;
			break;
		}
		if (f[0] === CAP_FRAME.entry && L >= 8) {
			entries.push({
				ms: (f[1] | (f[2] << 8) | (f[3] << 16) | (f[4] << 24)) >>> 0,
				slot: f[5],
				rid: f[6],
				nb: f[7],
				bytes: [...f.slice(8, 8 + f[7])],
			});
		}
	}
	return { entries, consumed: i, done };
}
