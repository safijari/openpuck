// Lizard (desktop) binding map. Mirrors firmware lizard_map.h -- the output
// types and the per-type payload (od[]) layout must match those constants
// exactly. From docs/index.html:584-631 and :1587-1740.

export const LZ_MAX = 32;

/** Output types. Values are the wire encoding and must match lizard_map.h. */
export const LZO = {
	NONE: 0,
	KBD: 1,
	MBTN: 2,
	AXIS: 3,
	SCROLL: 4,
	CONSUMER: 5,
} as const;

export const LZ_OUT_LABELS: [number, string][] = [
	[LZO.NONE, '(disabled)'],
	[LZO.KBD, 'Keyboard key'],
	[LZO.MBTN, 'Mouse button'],
	[LZO.AXIS, 'Mouse move'],
	[LZO.SCROLL, 'Scroll wheel'],
	[LZO.CONSUMER, 'Media key'],
];

/** Controller input bits (triton.h TB_* plus virtual stick-deflection bits). Single-bit triggers only. */
export const LZ_BTNS: [number, string][] = [
	[0x1, 'A'],
	[0x2, 'B'],
	[0x4, 'X'],
	[0x8, 'Y'],
	[0x10, 'QAM (• • •)'],
	[0x40, 'View'],
	[0x4000, 'Menu'],
	[0x10000, 'Steam'],
	[0x20, 'R3 (stick click)'],
	[0x8000, 'L3 (stick click)'],
	[0x80, 'R4 (back upper-right)'],
	[0x100, 'R5 (back lower-right)'],
	[0x20000, 'L4 (back upper-left)'],
	[0x40000, 'L5 (back lower-left)'],
	[0x200, 'RB (bumper)'],
	[0x80000, 'LB (bumper)'],
	[0x800000, 'R2 (trigger pull)'],
	[0x8000000, 'L2 (trigger pull)'],
	[0x2000, 'D-pad Up'],
	[0x400, 'D-pad Down'],
	[0x1000, 'D-pad Left'],
	[0x800, 'D-pad Right'],
	[0x400000, 'Right pad click'],
	[0x4000000, 'Left pad click'],
	[0x200000, 'Right pad touch'],
	[0x2000000, 'Left pad touch'],
	[0x10000000, 'L-stick → right'],
	[0x20000000, 'L-stick → left'],
	[0x40000000, 'L-stick → down'],
	[0x80000000, 'L-stick → up'],
];

/** Keyboard modifier bits, held in od[0] for a KBD output. */
export const LZ_MODS: [number, string][] = [
	[0x01, 'Ctrl'],
	[0x02, 'Shift'],
	[0x04, 'Alt'],
	[0x08, 'Win/⌘'],
];

/** Curated HID keycodes for od[1]. 0 means no key, i.e. a modifier-only binding. */
export const LZ_KEYS: [number, string][] = (() => {
	const keys: [number, string][] = [[0, '— none —']];
	'ABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('').forEach((c, i) => keys.push([0x04 + i, c]));
	'1234567890'.split('').forEach((c, i) => keys.push([0x1e + i, c]));
	const named: [number, string][] = [
		[0x28, 'Enter'],
		[0x29, 'Esc'],
		[0x2a, 'Backspace'],
		[0x2b, 'Tab'],
		[0x2c, 'Space'],
		[0x4f, 'Arrow Right'],
		[0x50, 'Arrow Left'],
		[0x51, 'Arrow Down'],
		[0x52, 'Arrow Up'],
		[0x4a, 'Home'],
		[0x4d, 'End'],
		[0x4b, 'Page Up'],
		[0x4e, 'Page Down'],
		[0x49, 'Insert'],
		[0x4c, 'Delete'],
		[0x2d, '- _'],
		[0x2e, '= +'],
		[0x46, 'Print Screen'],
	];
	named.forEach((k) => keys.push(k));
	for (let i = 0; i < 12; i++) keys.push([0x3a + i, `F${i + 1}`]);
	return keys;
})();

export const LZ_MBTNS: [number, string][] = [
	[1, 'Left click'],
	[2, 'Right click'],
	[4, 'Middle click'],
];

export const LZ_AXIS_SRC: [number, string][] = [
	[0, 'Right trackpad'],
	[1, 'Left stick'],
	[2, 'Gyro'],
];

export const LZ_GYRO_ACT: [number, string][] = [
	[0, 'Always'],
	[1, 'While right pad touched'],
	[2, 'While left stick deflected'],
	[3, 'While hold-button held'],
];

export const LZ_CONSUMER: [number, string][] = [
	[1, 'Volume +'],
	[2, 'Volume −'],
];

export interface LizardBinding {
	outType: number;
	/** Output payload; layout depends on outType. Always 7 bytes on the wire. */
	od: number[];
	trig: number;
	hold: number;
}

/** Lizard opcodes. */
export const LZ_OP = {
	dump: 0x11,
	setBinding: 0x12,
	beginEdit: 0x13,
	commit: 0x14,
	reset: 0x15,
} as const;

/** Marker of a lizard-map dump: [0xAA][count][count * 16 bytes]. */
export const LIZARD_MARK = 0xaa;

/** Analog outputs are driven by a source, not a button, so they have no trigger. */
export const isAnalog = (b: LizardBinding) => b.outType === LZO.AXIS || b.outType === LZO.SCROLL;

/** Digital outputs fire on a trigger and must have one -- see filterSavable. */
export const isDigital = (b: LizardBinding) =>
	b.outType === LZO.KBD || b.outType === LZO.MBTN || b.outType === LZO.CONSUMER;

/**
 * A digital binding with neither a trigger nor a hold fires unconditionally in
 * the firmware (the trigMask==0 guard is skipped) -- it would type or click
 * forever. Drop those rows so a half-finished binding cannot brick desktop
 * input.
 */
export function filterSavable(bindings: LizardBinding[]): LizardBinding[] {
	return bindings.filter((b) => b.outType !== LZO.NONE && (!isDigital(b) || b.trig >>> 0 || b.hold >>> 0));
}

/** Payload defaults when the output type changes. */
export function defaultPayload(outType: number): number[] {
	if (outType === LZO.MBTN || outType === LZO.CONSUMER) return [1, 0, 0, 0, 0, 0, 0];
	return [0, 0, 0, 0, 0, 0, 0];
}

export function btnLabel(mask: number): string {
	const m = LZ_BTNS.find((b) => b[0] === mask >>> 0);
	if (m) return m[1];
	return mask ? `0x${(mask >>> 0).toString(16)}` : '';
}

/** Decode a [0xAA][count][count*16] frame body into bindings. */
export function decodeBindings(acc: Uint8Array, count: number): LizardBinding[] {
	const out: LizardBinding[] = [];
	for (let b = 0; b < count; b++) {
		const q = 2 + b * 16;
		out.push({
			outType: acc[q],
			od: [...acc.slice(q + 1, q + 8)],
			trig: (acc[q + 8] | (acc[q + 9] << 8) | (acc[q + 10] << 16) | (acc[q + 11] << 24)) >>> 0,
			hold: (acc[q + 12] | (acc[q + 13] << 8) | (acc[q + 14] << 16) | (acc[q + 15] << 24)) >>> 0,
		});
	}
	return out;
}

/** Encode one binding as the op-0x12 payload. */
export function encodeBinding(index: number, b: LizardBinding): number[] {
	const od = (b.od ?? []).slice(0, 7);
	while (od.length < 7) od.push(0);
	const t = b.trig >>> 0;
	const h = b.hold >>> 0;
	return [
		LZ_OP.setBinding,
		index,
		b.outType,
		...od.map((x) => x & 0xff),
		t & 0xff,
		(t >>> 8) & 0xff,
		(t >>> 16) & 0xff,
		(t >>> 24) & 0xff,
		h & 0xff,
		(h >>> 8) & 0xff,
		(h >>> 16) & 0xff,
		(h >>> 24) & 0xff,
	];
}

const label = (table: [number, string][], v: number, fallback = '?') => table.find(([k]) => k === v)?.[1] ?? fallback;

/**
 * What fires the binding, as readable text: "A", "A + hold L4", or the analog
 * source for outputs that have no trigger.
 */
export function describeTrigger(b: LizardBinding): string {
	if (isAnalog(b)) {
		if (b.outType === LZO.SCROLL) return 'Left trackpad';
		const src = label(LZ_AXIS_SRC, b.od[0] ?? 0, 'source');
		// Gyro is the only source with an activation condition.
		if ((b.od[0] ?? 0) === 2) return `${src}, ${label(LZ_GYRO_ACT, b.od[1] ?? 0).toLowerCase()}`;
		return src;
	}
	const trig = btnLabel(b.trig);
	const hold = btnLabel(b.hold);
	if (!trig && !hold) return '';
	if (trig && hold) return `${trig} + hold ${hold}`;
	return trig || `hold ${hold}`;
}

/** What the binding does, as readable text: "Ctrl + C", "Left click", "Volume +". */
export function describeOutput(b: LizardBinding): string {
	switch (b.outType) {
		case LZO.KBD: {
			const mods = LZ_MODS.filter(([bit]) => b.od[0] & bit).map(([, name]) => name);
			const key = b.od[1] ? label(LZ_KEYS, b.od[1], `0x${b.od[1].toString(16)}`) : '';
			if (!mods.length && !key) return 'no key set';
			return [...mods, key].filter(Boolean).join(' + ');
		}
		case LZO.MBTN:
			return label(LZ_MBTNS, b.od[0] || 1);
		case LZO.AXIS:
			return 'Mouse move';
		case LZO.SCROLL:
			return 'Scroll wheel';
		case LZO.CONSUMER:
			return label(LZ_CONSUMER, b.od[0] || 1);
		default:
			return 'disabled';
	}
}

/**
 * Why a row would be dropped on save, or null if it is fine. Surfacing this on
 * the row means an incomplete binding is visible while editing, rather than
 * only appearing as a "skipped N" note in the log after saving.
 */
export function bindingProblem(b: LizardBinding): string | null {
	if (b.outType === LZO.NONE) return 'Disabled — will not be saved';
	if (isDigital(b) && !(b.trig >>> 0) && !(b.hold >>> 0)) return 'No trigger — will not be saved';
	if (b.outType === LZO.KBD && !(b.od[0] & 0x0f) && !b.od[1]) return 'No key or modifier set';
	return null;
}
