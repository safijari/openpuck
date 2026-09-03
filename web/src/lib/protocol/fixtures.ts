// A synthetic v17 status blob matching what the live panel reported from a real
// puck (fw 0.9.31, PS5 DualSense mode, one controller up at 93% / -43 dBm,
// 250 polls/s, 246 delivered/s, loop running, usbd stack 97 words, xtal/xtal
// clocks, 1000 us/ms, last reset "reboot"). Used to develop the UI without
// hardware attached and as the parseBlob test input.

import { LZO, type LizardBinding } from './lizard';

export interface BlobSpec {
	protocol: number;
	mode: number;
	build: string;
	battery: number;
	rssi: number;
	polls: number;
	delivered: number;
	bonded: number;
}

/** Values read off a real puck, so the fixture matches something that existed. */
export const LIVE_V17: BlobSpec = {
	protocol: 17,
	mode: 5,
	build: '0.9.31',
	battery: 93,
	rssi: 43,
	polls: 250,
	delivered: 246,
	bonded: 1,
};

/** Builds a payload as readFrame() hands it over: header already stripped. */
export function buildBlob(spec: BlobSpec = LIVE_V17): Uint8Array {
	// v17 payloads run to 179 bytes; the v18+ tails (D-pad chords, gyro map,
	// pad-stick, rumble) are deliberately absent so the capability gates are
	// exercised the way an older puck exercises them.
	const p = new Uint8Array(179);
	const w16 = (o: number, v: number) => {
		p[o] = v & 0xff;
		p[o + 1] = (v >> 8) & 0xff;
	};
	const w32 = (o: number, v: number) => {
		for (let i = 0; i < 4; i++) p[o + i] = (v >>> (8 * i)) & 0xff;
	};

	p[0] = spec.protocol;
	p[1] = spec.mode;
	p[2] = 32; // mouse divisor
	p[3] = 40; // mouse friction
	p[10] = 0; // active slot
	p[11] = 1; // link up
	w16(12, spec.delivered);
	p[14] = 40; // intended poll period / 100us -> 4000us
	w16(15, spec.delivered); // new reports/s
	p[22] = 0; // persist mode off
	p[23] = 3; // back4+B -> Lizard
	p[24] = 1; // back4+X -> Xbox 360
	p[25] = 4; // back4+Y -> Switch Pro
	w16(26, spec.polls);
	w16(28, 3900); // loop period us
	p[30] = 4; // slowest stage: rflink
	w16(31, 1800);
	w16(33, 4000); // actual poll period
	p[35] = 0; // not a logging build
	p[36] = spec.battery;
	p[37] = spec.rssi;
	p[38] = 0; // git clean
	for (let i = 0; i < spec.build.length && i < 12; i++) p[39 + i] = spec.build.charCodeAt(i);

	// IMU: resting, gravity on Z. ~16384 = 1g on this +/-2g sensor.
	const w = (o: number, v: number) => w16(o, v < 0 ? v + 65536 : v);
	w(54, 107);
	w(56, -2371);
	w(58, 16323);

	p[60] = spec.bonded;
	p[61] = 1; // slot 0 up
	p[62] = spec.battery;
	p[63] = spec.rssi;

	// Per-emulated-type config: 4 types x 9 bytes at 73.
	for (let et = 0; et < 4; et++) {
		const q = 73 + et * 9;
		p[q] = 5; // L4
		p[q + 1] = 6; // R4
		p[q + 2] = 7; // L5
		p[q + 3] = 8; // R5
		p[q + 4] = 1; // QAM
		p[q + 5] = 0; // A/B swap off
		p[q + 6] = 1; // trackpad haptics on
		p[q + 7] = 0; // LED auto
		p[q + 8] = 1; // rumble on
	}

	p[109] = 6; // reset cause: reboot
	w32(110, 0x00000004);
	p[117] = 0; // crc fails
	p[118] = 4; // noRx
	p[119] = 0; // heal
	w16(120, 0); // relay/s
	p[122] = 2; // LF xtal
	p[123] = 2; // HF xtal
	w16(124, 1000); // us per ms
	p[126] = 0xff; // last boot was not a hang
	p[127] = 4; // current stage: rflink
	p[128] = 0; // no stall
	w16(129, 0); // ring faults
	w16(139, 97); // usbd stack free words

	// Per-slot counters (v13).
	w16(143, spec.polls);
	w16(145, spec.delivered);
	w16(147, spec.delivered);
	p[149] = 0;
	p[150] = 4;
	p[151] = 0;

	return p;
}

/** A few representative bindings, for developing the editor without hardware. */
export const FIXTURE_LIZARD: LizardBinding[] = [
	// Ctrl+C on A.
	{ outType: LZO.KBD, od: [0x01, 0x06, 0, 0, 0, 0, 0], trig: 0x1, hold: 0 },
	// Left click on the right pad click.
	{ outType: LZO.MBTN, od: [1, 0, 0, 0, 0, 0, 0], trig: 0x400000, hold: 0 },
	// Gyro aiming while the right pad is touched.
	{ outType: LZO.AXIS, od: [2, 1, 0, 0, 0, 0, 0], trig: 0, hold: 0 },
	// Volume up on D-pad Up.
	{ outType: LZO.CONSUMER, od: [1, 0, 0, 0, 0, 0, 0], trig: 0x2000, hold: 0 },
];

/** A ReversePuck 0xAC frame: two paired pucks, one live, RF link up. */
export function buildDongleFrame(): Uint8Array {
	const pucks = [
		{ slot: 0, alive: true, serial: 'PUCK-A1B2' },
		{ slot: 1, alive: false, serial: 'PUCK-C3D4' },
	];
	const p = new Uint8Array(3 + pucks.length * 26);
	p[0] = 0xac;
	p[1] = 0b11; // forwarding + link up
	p[2] = pucks.length;
	pucks.forEach((k, i) => {
		const q = 3 + i * 26;
		p[q] = k.slot;
		p[q + 1] = k.alive ? 1 : 0;
		for (let n = 0; n < 4; n++) p[q + 2 + n] = 0xa0 + n + i;
		for (let n = 0; n < 4; n++) p[q + 6 + n] = 0xb0 + n + i;
		for (let n = 0; n < k.serial.length; n++) p[q + 10 + n] = k.serial.charCodeAt(n);
	});
	return p;
}
