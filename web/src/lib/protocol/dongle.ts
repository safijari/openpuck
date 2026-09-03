// ReversePuck: the mirror-image dongle (28DE:1302). It emulates a Steam
// Controller to whichever puck it is paired with, so its status frame (0xAC)
// lists paired pucks rather than controller slots.
// From docs/index.html:1186-1226.

export const DONGLE_OP = {
	/** Un-bond a paired puck by slot. */
	removePuck: 0x30,
} as const;

/** Bytes per bond record in the 0xAC payload. */
const REC = 26;

export interface PairedPuck {
	slot: number;
	alive: boolean;
	puuid: Uint8Array;
	iuuid: Uint8Array;
	serial: string;
}

export interface DongleStatus {
	/** Currently relaying for a Steam Deck. */
	forwarding: boolean;
	linkUp: boolean;
	pucks: PairedPuck[];
}

/**
 * Serials arrive space-padded and NUL-padded in a fixed 16-byte field, so trim
 * at the first space and drop anything non-printable before showing it.
 */
function decodeSerial(raw: Uint8Array): string {
	let s = new TextDecoder('latin1').decode(raw);
	const z = s.indexOf(' ');
	if (z >= 0) s = s.slice(0, z);
	// eslint-disable-next-line no-control-regex
	return s.replace(/[^\x20-\x7e]/g, '').trim();
}

/** Decode a 0xAC frame: the dongle's link flags and its paired pucks. */
export function parseDongleStatus(p: Uint8Array): DongleStatus {
	const flags = p[1];
	const count = p[2];
	const pucks: PairedPuck[] = [];
	for (let i = 0; i < count; i++) {
		const q = 3 + i * REC;
		// A truncated frame must not yield half a record.
		if (q + REC > p.length) break;
		pucks.push({
			slot: p[q],
			alive: !!p[q + 1],
			puuid: p.slice(q + 2, q + 6),
			iuuid: p.slice(q + 6, q + 10),
			serial: decodeSerial(p.slice(q + 10, q + 26)),
		});
	}
	return { forwarding: !!(flags & 1), linkUp: !!(flags & 2), pucks };
}

export const hexId = (a: Uint8Array) => [...a].map((x) => x.toString(16).padStart(2, '0')).join('');
