import { describe, expect, it } from 'vitest';
import {
	APP_BASE,
	compareVersions,
	crc32,
	errText,
	FWUP_MAX_IMG,
	imagePanelUpdatable,
	maxSends,
	parseAck,
	u32le,
	uf2ToImage,
	UF2_FAMILY,
	UF2_MAGIC0,
	UF2_MAGIC1,
	UF2_MAGIC_END,
} from '../firmware';

interface BlockOpts {
	addr?: number;
	len?: number;
	flags?: number;
	family?: number;
	magic0?: number;
	magicEnd?: number;
	fill?: number;
}

/** Build one 512-byte UF2 block. */
function block(o: BlockOpts = {}): Uint8Array {
	const { addr = APP_BASE, len = 256, flags = 0x00002000, family = UF2_FAMILY, fill = 0xaa } = o;
	const b = new Uint8Array(512);
	const dv = new DataView(b.buffer);
	dv.setUint32(0, o.magic0 ?? UF2_MAGIC0, true);
	dv.setUint32(4, UF2_MAGIC1, true);
	dv.setUint32(8, flags, true);
	dv.setUint32(12, addr, true);
	dv.setUint32(16, len, true);
	dv.setUint32(28, family, true);
	for (let i = 0; i < len; i++) b[32 + i] = fill;
	dv.setUint32(508, o.magicEnd ?? UF2_MAGIC_END, true);
	return b;
}

const uf2 = (...blocks: Uint8Array[]) => {
	const out = new Uint8Array(blocks.length * 512);
	blocks.forEach((b, i) => out.set(b, i * 512));
	return out.buffer;
};

describe('uf2ToImage', () => {
	it('flattens blocks into a contiguous image', () => {
		const img = uf2ToImage(uf2(block({ addr: APP_BASE, fill: 0x11 }), block({ addr: APP_BASE + 256, fill: 0x22 })));
		expect(img).toHaveLength(512);
		expect(img[0]).toBe(0x11);
		expect(img[256]).toBe(0x22);
	});

	it('pads the image to a whole number of words', () => {
		// The firmware stages whole words, so a 254-byte payload must round up.
		const img = uf2ToImage(uf2(block({ len: 254 })));
		expect(img.length % 4).toBe(0);
		expect(img).toHaveLength(256);
	});

	it('fills gaps between blocks with 0xFF', () => {
		const img = uf2ToImage(uf2(block({ addr: APP_BASE, len: 4 }), block({ addr: APP_BASE + 512, len: 4 })));
		expect(img[100]).toBe(0xff);
	});

	it('skips NOT_MAIN_FLASH blocks', () => {
		const img = uf2ToImage(uf2(block({ addr: APP_BASE, fill: 0x11 }), block({ addr: 0x1000, flags: 0x1 })));
		expect(img).toHaveLength(256);
	});

	it('rejects a file that is not a multiple of 512', () => {
		expect(() => uf2ToImage(new Uint8Array(500).buffer)).toThrow(/multiple of 512/);
	});

	it('rejects an empty file', () => {
		expect(() => uf2ToImage(new ArrayBuffer(0))).toThrow(/not a UF2/);
	});

	it('rejects bad block magic', () => {
		expect(() => uf2ToImage(uf2(block({ magic0: 0xdeadbeef })))).toThrow(/bad UF2 block magic/);
		expect(() => uf2ToImage(uf2(block({ magicEnd: 0xdeadbeef })))).toThrow(/bad UF2 block magic/);
	});

	it('rejects a UF2 for another chip family', () => {
		// Flashing an RP2040 image would brick the puck, so the family is checked.
		expect(() => uf2ToImage(uf2(block({ family: 0xe48bff56 })))).toThrow(/is not nRF52840/);
	});

	it('ignores the family when the flag says it is absent', () => {
		expect(() => uf2ToImage(uf2(block({ flags: 0, family: 0xe48bff56 })))).not.toThrow();
	});

	it('rejects an image that does not start at the app region', () => {
		// A bootloader or SoftDevice image would overwrite the wrong flash.
		expect(() => uf2ToImage(uf2(block({ addr: 0x1000 })))).toThrow(/not the app region/);
	});

	it('rejects an image over the staged-update cap', () => {
		const blocks = [block({ addr: APP_BASE, len: 1 }), block({ addr: APP_BASE + FWUP_MAX_IMG + 4, len: 1 })];
		expect(() => uf2ToImage(uf2(...blocks))).toThrow(/over the 384 KiB staged-update cap/);
	});

	it('rejects an out-of-range payload size', () => {
		expect(() => uf2ToImage(uf2(block({ len: 0 })))).toThrow(/bad UF2 payload size/);
		expect(() => uf2ToImage(uf2(block({ len: 477 })))).toThrow(/bad UF2 payload size/);
	});

	it('rejects a UF2 with no flash data', () => {
		expect(() => uf2ToImage(uf2(block({ flags: 0x1 })))).toThrow(/no flash data/);
	});
});

describe('crc32', () => {
	it('matches the IEEE reference vector', () => {
		// "123456789" -> 0xCBF43926 for CRC-32/ISO-HDLC, which fw_update.cpp uses.
		expect(crc32(new TextEncoder().encode('123456789'))).toBe(0xcbf43926);
	});

	it('returns 0 for empty input', () => {
		expect(crc32(new Uint8Array(0))).toBe(0);
	});

	it('stays unsigned for high-bit results', () => {
		expect(crc32(new Uint8Array([0]))).toBeGreaterThanOrEqual(0);
	});
});

describe('wire helpers', () => {
	it('encodes little-endian u32', () => {
		expect(u32le(0x12345678)).toEqual([0x78, 0x56, 0x34, 0x12]);
		expect(u32le(0xffffffff)).toEqual([0xff, 0xff, 0xff, 0xff]);
	});

	it('returns the LAST ack in a buffer', () => {
		// A retried command double-acks; only the newest state is true.
		const d = new Uint8Array([0xab, 5, 0, 0, 0, 0, 0, 0xab, 5, 0, 0x80, 0, 0, 0]);
		expect(parseAck(d)).toEqual({ status: 0, off: 0x80 });
	});

	it('returns null when there is no ack', () => {
		expect(parseAck(new Uint8Array([1, 2, 3]))).toBeNull();
	});

	it('names error statuses and falls back for unknown ones', () => {
		expect(errText(4)).toBe('staged bytes failed CRC verify');
		expect(errText(99)).toBe('code 99');
	});

	it('allows a retry budget above one send per chunk', () => {
		expect(maxSends(1280)).toBeGreaterThan(1280 / 128);
	});
});

describe('imagePanelUpdatable', () => {
	it('detects the tag that keeps panel updates working', () => {
		expect(imagePanelUpdatable(new TextEncoder().encode('xx OPK-FWUP-v1 xx'))).toBe(true);
	});

	it('reports an untagged image, which would end panel updates', () => {
		expect(imagePanelUpdatable(new TextEncoder().encode('nothing here'))).toBe(false);
	});

	it('does not throw on arbitrary binary', () => {
		expect(imagePanelUpdatable(new Uint8Array([0xff, 0x00, 0x80, 0xfe]))).toBe(false);
	});
});

describe('compareVersions', () => {
	it('orders releases', () => {
		expect(compareVersions('0.9.31', '0.9.6')).toBeGreaterThan(0);
		expect(compareVersions('0.9.6', '0.9.31')).toBeLessThan(0);
		expect(compareVersions('1.0.0', '0.9.99')).toBeGreaterThan(0);
		expect(compareVersions('0.9.3', '0.9.3')).toBe(0);
	});

	it('tolerates tags and missing components', () => {
		expect(compareVersions('v1.2.3', '1.2.3')).toBe(0);
		expect(compareVersions('1.2', '1.2.0')).toBe(0);
	});
});
