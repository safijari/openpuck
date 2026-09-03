// Pure decoder for the 0xA5 status blob.
//
// Lifted from applyBlob() in the hand-written panel (docs/index.html:902-1182),
// which decoded byte offsets and wrote to the DOM in the same pass. Only the
// decode half lives here; every offset, length guard and version gate below is
// carried over unchanged. Offsets are into the payload AFTER readFrame() strips
// the 2-byte [0xA5][len] header, so a firmware buffer index N lands at N-2 here.

export const MODE_NAMES = [
	'Steam (puck)',
	'Xbox 360',
	'Switch (HORIPAD)',
	'Lizard (always)',
	'Switch Pro + gyro',
	'PS5 DualSense',
	'HID gyro (DS4)',
	'PS5 (game/clean)',
	'DS4 (game/clean)',
	'PS3 (DualShock 3)',
	'Original Xbox',
	'DirectInput (sims)',
	'SInput (SDL native)',
] as const;

/** Loop stages, indexed by the firmware's stage id. */
export const STAGE_NAMES = [
	'webusb',
	'ctrl.task',
	'serial',
	'rfdiag',
	'rflink',
	'haptic',
	'led',
	'usbmount',
	'usbtx',
] as const;

/** Slowest-stage names. Shorter list than STAGE_NAMES -- the firmware only times these. */
export const WORST_NAMES = ['webusb', 'ctrl.task', 'serial', 'rfdiag', 'rflink', 'haptic', 'led'] as const;

/** Low-frequency clock source. 2 (xtal) is healthy; anything else is an RC fallback that drifts the RF schedule. */
export const LF_CLOCK = ['stopped', 'RC', 'xtal', 'synth'] as const;
/** High-frequency clock source; 2 (xtal) is healthy. */
export const HF_CLOCK = ['RC', '?', 'xtal', '?'] as const;

/** Reset causes; codes match RR_* in fault_diag.h. */
export const RESET_NAMES = [
	'unknown',
	'power-on',
	'pin/replug',
	'watchdog (hang)',
	'CPU lockup',
	'HARDFAULT',
	'reboot',
	'soft reset',
	'wake-from-off',
] as const;

/** watchdog / lockup / hardfault -- a real crash worth reporting (issue #72). */
export const RESET_FAULT_CODES = new Set([3, 4, 5]);

/** Firmware defaults for the D-pad chords: PS3, DS4 game, PS5 game, Switch HORIPAD. */
export const CHORD_DPAD_DEF = [9, 8, 7, 2];

/** Sentinel in the hang-stage byte meaning the last boot was not a hang. */
export const NO_HANG_STAGE = 0xff;

export interface SlotStats {
	polls: number;
	f1: number;
	newps: number;
	crc: number;
	norx: number;
	relay: number;
}

export interface SlotStatus {
	up: boolean;
	battery: number;
	rssi: number;
	stats?: SlotStats;
}

export interface TypeConfig {
	back: [number, number, number, number];
	qam: number;
	abSwap: boolean;
	padHaptics: boolean;
	led: number;
	rumble: boolean;
	/** Trackpad -> stick mapping, [left, right]. Only meaningful when caps.padStick. */
	padStick: [number, number];
}

/**
 * What each firmware revision added. The panel must keep working against older
 * pucks, so every one of these is checked before the field it guards is read.
 */
export interface Caps {
	/** v15: in-panel firmware updates. Older pucks need one manual UF2 DFU flash first. */
	panelUpdate: boolean;
	/** v16: the op-0x11 lizard-map dump exists. Sending it blind to older firmware wedges the endpoint. */
	lizard: boolean;
	/** v18: back4 + D-pad chords. */
	dpadChords: boolean;
	/** v19: Switch Pro gyro mapping (corrected vs legacy). */
	gyroMap: boolean;
	/** v20: per-type trackpad -> stick mapping. */
	padStick: boolean;
	/** v21: host-rumble shaping (style + strength). */
	rumble: boolean;
	/** v8: per-slot link status for up to 4 controllers. */
	slots: boolean;
	/** v13: each slot's own poll/delivery counters. */
	slotStats: boolean;
	/** v10/v17: per-emulated-type button config. */
	typeConfig: boolean;
}

export interface DeviceStatus {
	protocol: number;
	mode: number;
	modeName: string;
	mouse: { div: number; friction: number };
	link: { up: boolean; slot: number; battery: number; rssi: number };
	rates: {
		delivered: number;
		newReports: number;
		polls: number;
		relay: number | null;
		crc: number;
		norx: number;
		heal: number;
		ringFault: number;
	};
	loop: {
		periodUs: number;
		worstStage: string;
		worstUs: number;
		pollUs: number;
		pollIntendedUs: number;
		stage: string;
		stallMs: number;
		stalled: boolean;
		usbdStackWords: number | null;
	};
	clock: { lf: string; hf: string; usPerMs: number; lfBad: boolean; hfBad: boolean; usPerMsBad: boolean } | null;
	reset: {
		code: number;
		name: string;
		raw: number;
		isFault: boolean;
		hangStage: number;
		hangStageName: string | null;
		hangPC: number;
		hangLR: number;
	} | null;
	imu: { ax: number; ay: number; az: number; magnitude: number } | null;
	build: { id: string; dirty: boolean };
	persistMode: boolean;
	logEnabled: boolean;
	chords: { face: number[]; dpad: number[] };
	gyroLegacy: boolean;
	rumbleShaping: { style: number; scalePct: number };
	bondedCount: number;
	slots: (SlotStatus | null)[];
	types: TypeConfig[];
	caps: Caps;
}

const u16 = (p: Uint8Array, o: number) => p[o] | (p[o + 1] << 8);
const u32 = (p: Uint8Array, o: number) => (p[o] | (p[o + 1] << 8) | (p[o + 2] << 16) | (p[o + 3] << 24)) >>> 0;
const s16 = (p: Uint8Array, o: number) => {
	const v = u16(p, o);
	return v > 32767 ? v - 65536 : v;
};

/** Emulated controller types the firmware carries config for: Xbox, Switch, DS4, DS5. */
export const TYPE_COUNT = 4;

/**
 * Decode a status blob into typed state.
 *
 * Every read is guarded on both the protocol version and the payload length,
 * because an older puck simply sends a shorter frame -- reading past its end
 * would yield zeros that look like real settings.
 */
export function parseBlob(p: Uint8Array): DeviceStatus {
	const protocol = p[0];
	const mode = p[1];
	const up = !!p[11];

	const caps: Caps = {
		panelUpdate: protocol >= 15,
		lizard: protocol >= 16,
		dpadChords: protocol >= 18 && p.length > 183,
		gyroMap: protocol >= 19 && p.length > 184,
		padStick: protocol >= 20 && p.length > 192,
		rumble: protocol >= 21 && p.length > 193,
		slots: p.length >= 73,
		slotStats: p.length >= 179,
		typeConfig: p.length >= 73 + TYPE_COUNT * 9,
	};

	// Slot presence: the firmware sends all 4 slots but no bitmask saying which
	// are bonded, so a slot counts as present when it has ever reported
	// anything. A just-booted puck with everything offline reports nothing at
	// all, so fall back to the first bondedCount slots.
	const bondedCount = caps.slots ? p[60] : 0;
	const slots: (SlotStatus | null)[] = [null, null, null, null];
	if (caps.slots) {
		const seen: number[] = [];
		for (let s = 0; s < 4; s++) {
			const base = 61 + s * 3;
			if (p.length < base + 3) continue;
			const slot: SlotStatus = { up: !!p[base], battery: p[base + 1], rssi: p[base + 2] };
			if (slot.up || slot.battery || slot.rssi) {
				slots[s] = slot;
				seen.push(s);
			}
		}
		if (seen.length === 0) for (let s = 0; s < bondedCount && s < 4; s++) slots[s] = { up: false, battery: 0, rssi: 0 };

		if (caps.slotStats) {
			for (let s = 0; s < 4; s++) {
				if (!slots[s]) continue;
				const b = 143 + s * 9;
				slots[s]!.stats = {
					polls: u16(p, b),
					f1: u16(p, b + 2),
					newps: u16(p, b + 4),
					crc: p[b + 6],
					norx: p[b + 7],
					relay: p[b + 8],
				};
			}
		}
	}

	const types: TypeConfig[] = [];
	if (caps.typeConfig) {
		for (let et = 0; et < TYPE_COUNT; et++) {
			const q = 73 + et * 9;
			types.push({
				back: [p[q], p[q + 1], p[q + 2], p[q + 3]],
				qam: p[q + 4],
				abSwap: !!p[q + 5],
				padHaptics: !!p[q + 6],
				led: p[q + 7],
				rumble: !!p[q + 8],
				padStick: caps.padStick ? [p[185 + et * 2], p[185 + et * 2 + 1]] : [0, 0],
			});
		}
	}

	let clock: DeviceStatus['clock'] = null;
	if (p.length > 125) {
		const lf = p[122];
		const hf = p[123];
		const usPerMs = u16(p, 124);
		clock = {
			lf: LF_CLOCK[lf] ?? String(lf),
			hf: HF_CLOCK[hf] ?? String(hf),
			usPerMs,
			// 2 == xtal; anything else means the puck fell back to the RC
			// oscillator, which drifts the whole RF schedule.
			lfBad: lf !== 2,
			hfBad: hf !== 2,
			usPerMsBad: !!usPerMs && (usPerMs < 985 || usPerMs > 1015),
		};
	}

	let reset: DeviceStatus['reset'] = null;
	if (p.length > 113) {
		const code = p[109];
		const hangStage = p.length > 126 ? p[126] : NO_HANG_STAGE;
		reset = {
			code,
			name: RESET_NAMES[code] ?? `code ${code}`,
			raw: u32(p, 110),
			isFault: RESET_FAULT_CODES.has(code),
			hangStage,
			hangStageName: hangStage !== NO_HANG_STAGE ? (STAGE_NAMES[hangStage] ?? `stage ${hangStage}`) : null,
			hangPC: p.length > 134 ? u32(p, 131) : 0,
			hangLR: p.length > 138 ? u32(p, 135) : 0,
		};
	}

	let imu: DeviceStatus['imu'] = null;
	if (p.length >= 60) {
		const ax = s16(p, 54);
		const ay = s16(p, 56);
		const az = s16(p, 58);
		imu = { ax, ay, az, magnitude: Math.round(Math.hypot(ax, ay, az)) };
	}

	let build = '';
	for (let i = 39; i < 51 && i < p.length && p[i]; i++) build += String.fromCharCode(p[i]);

	// A stall under 200ms is just a slow iteration; at/above it the loop is
	// wedged and the watchdog (~8s) is counting down.
	const stallMs = p.length > 128 ? p[128] * 40 : 0;
	const curStage = p.length > 128 ? p[127] : 0;

	return {
		protocol,
		mode,
		modeName: MODE_NAMES[mode] ?? `mode ${mode}`,
		mouse: { div: p[2], friction: p[3] },
		link: { up, slot: p[10], battery: p.length > 36 ? p[36] : 0, rssi: p.length > 37 ? p[37] : 0 },
		rates: {
			delivered: u16(p, 12),
			newReports: p.length > 16 ? u16(p, 15) : 0,
			polls: p.length > 27 ? u16(p, 26) : 0,
			relay: p.length > 121 ? u16(p, 120) : null,
			crc: p.length > 117 ? p[117] : 0,
			norx: p.length > 118 ? p[118] : 0,
			heal: p.length > 119 ? p[119] : 0,
			ringFault: p.length > 130 ? u16(p, 129) : 0,
		},
		loop: {
			periodUs: p.length > 29 ? u16(p, 28) : 0,
			worstStage: WORST_NAMES[p.length > 30 ? p[30] : 0] ?? String(p[30]),
			worstUs: p.length > 32 ? u16(p, 31) : 0,
			pollUs: p.length > 34 ? u16(p, 33) : 0,
			pollIntendedUs: p.length > 14 ? p[14] * 100 : 0,
			stage: STAGE_NAMES[curStage] ?? `stage ${curStage}`,
			stallMs,
			stalled: stallMs >= 200,
			usbdStackWords: p.length > 140 ? u16(p, 139) : null,
		},
		clock,
		reset,
		imu,
		build: { id: build, dirty: p.length > 38 && !!p[38] },
		persistMode: p.length > 22 && !!p[22],
		logEnabled: p.length > 35 && !!p[35],
		chords: {
			face: p.length > 25 ? [p[23], p[24], p[25]] : [3, 1, 4],
			dpad: caps.dpadChords ? [p[180], p[181], p[182], p[183]] : CHORD_DPAD_DEF,
		},
		gyroLegacy: caps.gyroMap ? !!p[184] : false,
		// Strength is stored as percent/2; 200% is the shipped default.
		rumbleShaping: {
			style: caps.rumble ? p[193] : 0,
			scalePct: caps.rumble ? p[51] * 2 : 200,
		},
		bondedCount,
		slots,
		types,
		caps,
	};
}
