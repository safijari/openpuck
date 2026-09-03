// Per-emulated-type button config. The order must match the firmware's ET_*
// enum: Xbox=0, Switch=1, DS4=2, DS5=3. Each type lists only the remap targets
// that actually exist on that controller, so the panel can never offer a
// binding the console would reject.
//
// From docs/index.html:573-582.

const DPAD: Record<number, string> = {
	12: 'D-pad Up',
	13: 'D-pad Down',
	14: 'D-pad Left',
	15: 'D-pad Right',
};

export interface TypeDef {
	key: string;
	name: string;
	labels: Record<number, string>;
}

/** Per-type remap targets. Order must match the firmware's ET_* enum. */
export const TYPE_DEFS: TypeDef[] = [
	{
		key: 'XBOX',
		name: 'Xbox',
		labels: {
			1: 'A',
			2: 'B',
			3: 'X',
			4: 'Y',
			5: 'LB',
			6: 'RB',
			7: 'L3',
			8: 'R3',
			9: 'Back',
			10: 'Start',
			11: 'Guide',
			19: 'LT',
			20: 'RT',
			...DPAD,
		},
	},
	{
		key: 'SWITCH',
		name: 'Switch',
		labels: {
			1: 'A',
			2: 'B',
			3: 'X',
			4: 'Y',
			5: 'L',
			6: 'R',
			7: 'L-Stick',
			8: 'R-Stick',
			9: 'Minus',
			10: 'Plus',
			11: 'Home',
			19: 'ZL',
			20: 'ZR',
			...DPAD,
			18: 'Capture / Screenshot',
		},
	},
	{
		key: 'DS4',
		name: 'DS4',
		labels: {
			1: 'Cross',
			2: 'Circle',
			3: 'Square',
			4: 'Triangle',
			5: 'L1',
			6: 'R1',
			7: 'L3',
			8: 'R3',
			9: 'Create',
			10: 'Options',
			11: 'PS',
			19: 'L2',
			20: 'R2',
			...DPAD,
			16: 'Touchpad Click',
		},
	},
	{
		key: 'DS5',
		name: 'DS5',
		labels: {
			1: 'Cross',
			2: 'Circle',
			3: 'Square',
			4: 'Triangle',
			5: 'L1',
			6: 'R1',
			7: 'L3',
			8: 'R3',
			9: 'Create',
			10: 'Options',
			11: 'PS',
			19: 'L2',
			20: 'R2',
			...DPAD,
			16: 'Touchpad Click',
			17: 'Mute',
		},
	},
];

/** The four back paddles, in the order the firmware stores them. */
export const BACK_LABELS = [
	'L4 (back upper-left)',
	'R4 (back upper-right)',
	'L5 (back lower-left)',
	'R5 (back lower-right)',
];

/**
 * Trackpad -> joystick mapping (firmware PS_OFF / PS_LEFT / PS_RIGHT). While
 * mapped and touched the pad drives that stick; on release the stick
 * re-centres, and an untouched mapped pad leaves the physical stick in control.
 */
export const PAD_STICK_OPTS: [number, string][] = [
	[0, 'Off (touchpad)'],
	[1, 'Left stick'],
	[2, 'Right stick'],
];

/** Trackpad -> stick selects, indexed left then right. */
export const PAD_STICK_LABELS = ['Left trackpad → stick', 'Right trackpad → stick'];

/** Sorted remap targets for a type, with the leading "none"/"default" entry. */
export function targetOptions(def: TypeDef, includeNone: boolean): [number, string][] {
	const head: [number, string] = includeNone ? [0, '— none —'] : [0, 'Default (per-mode)'];
	const rest = Object.keys(def.labels)
		.map(Number)
		.sort((a, b) => a - b)
		.map((c) => [c, def.labels[c]] as [number, string]);
	return [head, ...rest];
}

/** Which emulated type a USB mode belongs to; -1 = a puck mode with no type. */
export function etypeForMode(m: number): number {
	switch (m) {
		case 1:
		case 10:
			return 0;
		case 2:
		case 4:
			return 1;
		case 6:
		case 8:
		case 9:
			return 2;
		case 5:
		case 7:
			return 3;
		default:
			return -1;
	}
}
