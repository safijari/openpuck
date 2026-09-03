// Config field ids for op 0x02 -> [0x02, field, value].
// Taken from the change handlers in docs/index.html:2299-2315 and
// buildTypeCfgs() at :670-704. These are wire values -- do not renumber.

export const FIELD = {
	mouseDiv: 1,
	mouseFriction: 2,
	persistMode: 16,
	/** back4 + B / X / Y */
	chordFace: [17, 18, 19],
	/** Rumble strength. Sent as percent/2, so the value must be even. */
	rumbleScale: 22,
	/** back4 + D-pad left / up / right / down (protocol >= 18) */
	chordDpad: [34, 35, 36, 37],
	/** Switch Pro gyro mapping: 0 corrected, 1 legacy (protocol >= 19) */
	swGyroMap: 38,
	rumbleStyle: 39,
} as const;

/** Per-emulated-type config block: 9 fields each, starting at 40. */
export const TYPE_FIELD0 = 40;
/** Fields per emulated type, so type et starts at TYPE_FIELD0 + et * TYPE_STRIDE. */
export const TYPE_STRIDE = 9;

/** Offsets within a type's block. */
export const TYPE_OFF = {
	back: 0, // 4 entries: back[0..3]
	qam: 4,
	abSwap: 5,
	padHaptics: 6,
	led: 7,
	rumble: 8,
} as const;

export const typeField = (et: number, off: number) => TYPE_FIELD0 + et * TYPE_STRIDE + off;

/** Per-type trackpad -> stick mapping, 2 per type (protocol >= 20). */
export const PAD_STICK_FIELD0 = 80;
export const padStickField = (et: number, pad: 0 | 1) => PAD_STICK_FIELD0 + et * 2 + pad;

/** Bare opcodes (no field byte). */
export const OP = {
	setMode: 0x00,
	status: 0x01,
	setField: 0x02,
	capture: 0x06,
	/** Re-init haptics, clearing a stuck buzz. */
	hapticClear: 0x07,
	/** Controller power-off (0x9F "off!", x3). */
	controllerOff: 0x08,
	stabilityBuzz: 0x0f,
	/** Reboot into serial DFU (adafruit-nrfutil). */
	dfuSerial: 0x0b,
	/** Reboot into the UF2 mass-storage bootloader. */
	dfuUf2: 0x0c,
	flightRecorder: 0x10,
	lizardDump: 0x11,
	rumbleTest: 0x16,
} as const;

/**
 * Destructive commands carry a literal payload as a guard, so a stray opcode
 * cannot trigger one. Factory erase is 0x0A + "ERS"; the full board wipe is
 * 0x25 + "WIPE".
 */
export const FACTORY_ERASE_CMD = [0x0a, 0x45, 0x52, 0x53];
export const WIPE_BOARD_CMD = [0x25, 0x57, 0x49, 0x50, 0x45];

/** Debug CDC is a config field, not an opcode: it auto-reverts after one boot. */
export const DEBUG_CDC_FIELD = 20;

/** Rumble styles; names track RUMBLE_STYLE_* in haptics.h, values are the wire values. */
export const RUMBLE_STYLES: [string, number][] = [
	['Normal (default)', 0],
	['Mono (both motors)', 1],
	['Heavy (low only)', 2],
	['Light (high only)', 3],
	['Swapped motors', 4],
	['Punchy (squared)', 5],
	['Soft (boost weak)', 6],
];

/** Strength presets. All even, since the wire value is percent/2. */
export const RUMBLE_SCALES = [50, 75, 100, 150, 200, 250, 300, 400, 500];

/** Switch Pro gyro mapping. Corrected trims the axes to match a genuine Pro Controller. */
export const GYRO_MAPS: [string, number][] = [
	['Corrected (default)', 0],
	['Legacy (untrimmed)', 1],
];
