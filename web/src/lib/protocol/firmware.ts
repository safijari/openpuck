// In-browser firmware update: parse a UF2, stream it into the puck's spare
// flash, have the puck verify it, then reboot to apply.
//
// Lifted from docs/index.html:1741-1864. Nothing here is armed until the puck
// has CRC-verified the staged image itself, so a failed or interrupted transfer
// leaves the running firmware untouched -- and even a power cut during the
// apply only leaves the puck in its UF2 bootloader for drag-and-drop recovery.

export const UF2_MAGIC0 = 0x0a324655;
/** Second UF2 block magic; both must match or the file is not a UF2. */
export const UF2_MAGIC1 = 0x9e5d5157;
/** Trailing magic at byte 508 of every 512-byte block. */
export const UF2_MAGIC_END = 0x0ab16f30;
/** nRF52840 family id. An image for another chip family would brick the puck, so it is checked. */
export const UF2_FAMILY = 0xada52840;
/** Start of the application region; anything else is not an OpenPuck app UF2. */
export const APP_BASE = 0x26000;
/** Staged-update cap: 384 KiB of spare flash. */
export const FWUP_MAX_IMG = 0x60000;
/** Payload bytes per chunk command. */
export const FWUP_CHUNK = 128;

/** Update opcodes: begin, stream a chunk, verify+commit, reboot to apply. */
export const FWUP_OP = {
	begin: 0x20,
	chunk: 0x21,
	verifyCommit: 0x22,
	reboot: 0x23,
} as const;

/** Marker of an update ack frame: [0xAB][5][status][nextOff u32 LE]. */
export const FWUP_ACK_MARK = 0xab;

/** Update status codes, indexed by the ack's status byte. */
export const FWUP_ERR = [
	'ok',
	'command out of sequence',
	"image too big for this puck's free flash",
	'offset resync',
	'staged bytes failed CRC verify',
	'staged image has no valid vector table',
];

/** Status 3 is not a failure: the firmware is telling us where to resume. */
export const FWUP_STATUS_RESYNC = 3;

export const errText = (status: number) => FWUP_ERR[status] ?? `code ${status}`;

/**
 * Flatten a UF2 into the contiguous image the firmware stages.
 * Throws with a specific message for every rejection, since "bad file" is
 * useless when the cause is a wrong family or a non-app base address.
 */
export function uf2ToImage(buf: ArrayBuffer): Uint8Array {
	if (!buf.byteLength || buf.byteLength % 512) throw new Error('not a UF2 (size is not a multiple of 512)');
	const dv = new DataView(buf);
	const blocks: { addr: number; len: number; off: number }[] = [];
	let lo = Infinity;
	let hi = 0;

	for (let off = 0; off < buf.byteLength; off += 512) {
		if (
			dv.getUint32(off, true) !== UF2_MAGIC0 ||
			dv.getUint32(off + 4, true) !== UF2_MAGIC1 ||
			dv.getUint32(off + 508, true) !== UF2_MAGIC_END
		) {
			throw new Error(`bad UF2 block magic at offset ${off}`);
		}
		const flags = dv.getUint32(off + 8, true);
		const addr = dv.getUint32(off + 12, true);
		const len = dv.getUint32(off + 16, true);
		if (flags & 0x00000001) continue; // NOT_MAIN_FLASH
		if (flags & 0x00002000) {
			const family = dv.getUint32(off + 28, true);
			if (family !== UF2_FAMILY) {
				throw new Error(`UF2 family 0x${family.toString(16)} is not nRF52840 (0xada52840)`);
			}
		}
		if (len < 1 || len > 476) throw new Error(`bad UF2 payload size ${len}`);
		blocks.push({ addr, len, off });
		lo = Math.min(lo, addr);
		hi = Math.max(hi, addr + len);
	}

	if (!blocks.length) throw new Error('UF2 contains no flash data');
	if (lo !== APP_BASE) {
		throw new Error(`image base 0x${lo.toString(16)} is not the app region (0x26000) — not an OpenPuck app UF2`);
	}
	if (hi - lo > FWUP_MAX_IMG) {
		throw new Error(
			`image is ${Math.round((hi - lo) / 1024)} KiB — over the 384 KiB staged-update cap; ` +
				'flash it via UF2 DFU + drag-and-drop',
		);
	}

	// 0xFF fill and a word-aligned length: the firmware stages whole words.
	const img = new Uint8Array((hi - lo + 3) & ~3).fill(0xff);
	for (const b of blocks) img.set(new Uint8Array(buf, b.off + 32, b.len), b.addr - lo);
	return img;
}

/** CRC32 (IEEE, reflected, init/xorout 0xFFFFFFFF). Must match crc32Step/crc32Flash in fw_update.cpp. */
export function crc32(u8: Uint8Array): number {
	let c = 0xffffffff;
	for (let i = 0; i < u8.length; i++) {
		c ^= u8[i];
		for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xedb88320 & -(c & 1));
	}
	return ~c >>> 0;
}

export const u32le = (v: number) => [v & 0xff, (v >>> 8) & 0xff, (v >>> 16) & 0xff, (v >>> 24) & 0xff];

/**
 * Firmware from this branch onward embeds FWUP_TAG. An image without it still
 * flashes fine, but the puck it leaves behind cannot do panel updates -- so the
 * caller warns before that lock-yourself-out-to-drag-and-drop move. Every
 * release up to 0.9.6 predates panel updates.
 */
export function imagePanelUpdatable(image: Uint8Array): boolean {
	return new TextDecoder('latin1').decode(image).includes('OPK-FWUP-v1');
}

export interface FwupAck {
	status: number;
	off: number;
}

/** Find the LAST ack in a buffer: a retried command double-acks and only the newest state is true. */
export function parseAck(d: Uint8Array): FwupAck | null {
	let ack: FwupAck | null = null;
	for (let i = 0; i + 6 < d.length; i++) {
		if (d[i] === FWUP_ACK_MARK && d[i + 1] === 5) {
			ack = { status: d[i + 2], off: (d[i + 3] | (d[i + 4] << 8) | (d[i + 5] << 16) | (d[i + 6] << 24)) >>> 0 };
		}
	}
	return ack;
}

/** Cap on chunk sends before declaring the transfer non-convergent. */
export const maxSends = (imageLen: number) => Math.ceil(imageLen / FWUP_CHUNK) * 2 + 64;

/** Compare two "1.2.3" versions. Returns >0 if a is newer. */
export function compareVersions(a: string, b: string): number {
	const t = (s: string) => (s.match(/\d+/g) ?? []).slice(0, 3).map(Number);
	const [x, y] = [t(a), t(b)];
	for (let i = 0; i < 3; i++) {
		if ((x[i] ?? 0) !== (y[i] ?? 0)) return (x[i] ?? 0) - (y[i] ?? 0);
	}
	return 0;
}
