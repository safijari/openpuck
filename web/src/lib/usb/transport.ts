// WebUSB transport. Lifted from docs/index.html:760-901 with the DOM writes
// pulled out -- callbacks report state changes so the caller owns the UI.
//
// The framing, the interface-selection rules and the timing constants here are
// reverse-engineered and load-bearing; treat them as fixed unless a real puck
// says otherwise.

import { parseAck, type FwupAck } from '$lib/protocol/firmware';

/**
 * Every VID an OpenPuck can enumerate under. The picker and auto-reconnect can
 * only see a device whose VID is listed here, so a new mode with a new identity
 * MUST be added (1209 = DirectInput, 2E8A = SInput).
 */
export const USB_FILTERS: USBDeviceFilter[] = [
	{ vendorId: 0x28de },
	{ vendorId: 0x045e },
	{ vendorId: 0x057e },
	{ vendorId: 0x0f0d },
	{ vendorId: 0x054c },
	{ vendorId: 0x1209 },
	{ vendorId: 0x2e8a },
];

/** ReversePuck controller dongle, as opposed to a puck (0x1304 / real). */
export const DONGLE_PID = 0x1302;

/** Status blob. */
export const MARK_STATUS = 0xa5;
/** Bond dump, for backup/clone. */
export const MARK_BONDS = 0xa7;
/** Flight-recorder stream. */
export const MARK_FLIGHT = 0xa8;
/** Live wedge report, pushed from the firmware's SOF callback. */
export const MARK_WEDGE = 0xa9;
/** Capture ring entries. */
export const MARK_CAPTURE = 0xa6;
/** Paired-pucks list (dongle only). */
export const MARK_PAIRED = 0xac;
/** Lizard map dump. */
export const MARK_LIZARD = 0xaa;

export interface WedgeReport {
	stage: number;
	stallMs: number;
}

/**
 * What the transport needs to report progress, supplied by the caller. Keeping
 * it an interface rather than importing the log store keeps this module below
 * the state layer, so it stays usable (and testable) without one.
 */
export interface TransportLogger {
	info: (m: string) => void;
	ok: (m: string) => void;
	error: (m: string) => void;
}

export interface TransportEvents {
	onOpen: (info: { isDongle: boolean; serial: string; vendorId: number; productId: number }) => void;
	onGone: () => void;
	/** Fires whenever a 0xA9 frame is seen in any read, not just a targeted one. */
	onWedge: (w: WedgeReport) => void;
}

/**
 * The WebUSB connection to a puck.
 *
 * Reports through callbacks and a supplied logger rather than touching UI
 * state, which is what keeps this below the state layer -- and testable
 * without one.
 */
export class Transport {
	private dev: USBDevice | null = null;
	private epIn = 0;
	private epOut = 0;
	private ifNum = 0;
	private events: TransportEvents;
	private log: TransportLogger;

	constructor(events: TransportEvents, log: TransportLogger) {
		this.events = events;
		this.log = log;
	}

	get device() {
		return this.dev;
	}

	get connected() {
		return this.dev !== null;
	}

	/** Wire up auto-reconnect. Call once. */
	listen() {
		if (typeof navigator === 'undefined' || !navigator.usb) return;
		navigator.usb.addEventListener('connect', () => {
			if (!this.dev) void this.autoConnect();
		});
		navigator.usb.addEventListener('disconnect', (e) => {
			if ((e as USBConnectionEvent).device === this.dev) this.handleGone();
		});
	}

	/** Manual connect: shows the Chrome picker, needed once to authorize the device. */
	async connect(): Promise<boolean> {
		try {
			const d = await navigator.usb.requestDevice({ filters: USB_FILTERS });
			return await this.open(d);
		} catch (e) {
			this.log.error(`connect failed: ${(e as Error).message}`);
			return false;
		}
	}

	/** No-picker reconnect for a device already authorized in this browser. */
	async autoConnect(): Promise<boolean> {
		if (this.dev) return true;
		try {
			const ds = await navigator.usb.getDevices();
			const d = ds.find((x) => USB_FILTERS.some((f) => f.vendorId === x.vendorId));
			if (d) return await this.open(d);
		} catch {
			// Nothing authorized yet, or the browser refused. Not an error.
		}
		return false;
	}

	/** Acquire an already-chosen device. Shared by the picker and auto-reconnect. */
	async open(d: USBDevice): Promise<boolean> {
		try {
			this.dev = d;
			await d.open();
			if (d.configuration === null) await d.selectConfiguration(1);

			// The vendor interface is class 0xFF with a bulk pair. Subclass 0x5D
			// is the Xbox 360 control interface, which is also vendor-class but
			// is not ours -- claiming it would break the gamepad.
			let found: { ifNum: number; epIn: number; epOut: number } | null = null;
			for (const itf of d.configuration!.interfaces) {
				const a = itf.alternate;
				if (a.interfaceClass !== 0xff) continue;
				if (a.interfaceSubclass === 0x5d) continue;
				let bin = 0;
				let bout = 0;
				for (const e of a.endpoints) {
					if (e.type !== 'bulk') continue;
					if (e.direction === 'in') bin = e.endpointNumber;
					else bout = e.endpointNumber;
				}
				if (bin && bout) {
					found = { ifNum: itf.interfaceNumber, epIn: bin, epOut: bout };
					break;
				}
			}
			if (!found) {
				this.log.error(
					'no WebUSB bulk vendor interface — reflash firmware with WebUSB enabled, or another app may be holding the device',
				);
				this.dev = null;
				return false;
			}

			this.ifNum = found.ifNum;
			this.epIn = found.epIn;
			this.epOut = found.epOut;
			await d.claimInterface(this.ifNum);
			// CDC SET_CONTROL_LINE_STATE with DTR asserted; the firmware waits
			// for it before it will talk.
			await d.controlTransferOut({
				requestType: 'class',
				recipient: 'interface',
				request: 0x22,
				value: 0x01,
				index: this.ifNum,
			});

			const isDongle = d.productId === DONGLE_PID;
			const serial = d.serialNumber ?? '?';
			this.log.ok(
				`connected — ${isDongle ? 'ReversePuck' : 'puck'} ${serial} ` +
					`(VID ${d.vendorId.toString(16)} PID ${d.productId.toString(16)} iface ${this.ifNum})`,
			);
			this.events.onOpen({
				isDongle,
				serial,
				vendorId: d.vendorId,
				productId: d.productId,
			});
			return true;
		} catch (e) {
			this.log.error(`connect failed: ${(e as Error).message}`);
			this.dev = null;
			return false;
		}
	}

	private handleGone() {
		// Only a real disconnect counts. Without this, anything that re-entered
		// this path with no device attached (a hot reload in dev, a doubled
		// event) would queue a phantom reset into the hang log.
		if (!this.dev) return;
		this.dev = null;
		this.events.onGone();
		// The puck re-enumerates after a reboot. The "connect" event usually
		// catches it, but poll too in case the event is missed.
		let tries = 0;
		const iv = setInterval(async () => {
			if (this.dev || tries++ > 40) {
				clearInterval(iv);
				return;
			}
			if (await this.autoConnect()) clearInterval(iv);
		}, 500);
	}

	async send(bytes: number[]): Promise<void> {
		if (!this.dev) return;
		try {
			await this.dev.transferOut(this.epOut, new Uint8Array(bytes));
		} catch (e) {
			this.log.error(`write err: ${(e as Error).message}`);
		}
	}

	/**
	 * Read one framed reply, returning the payload with the 2-byte
	 * [marker][len] header stripped. Several frame types share this framing and
	 * are told apart by the marker.
	 *
	 * readLen is overridable: the dongle's 0xAC paired-pucks list reaches 213 B
	 * for 8 pucks and needs 256.
	 */
	async readFrame(marker: number, minLen = 0, readLen = 256): Promise<Uint8Array | null> {
		if (!this.dev) return null;
		try {
			const r = await this.dev.transferIn(this.epIn, readLen);
			if (r.status !== 'ok' || !r.data || r.data.byteLength < 2) return null;
			const d = new Uint8Array(r.data.buffer);

			// The live wedge reporter (0xA9) is emitted from the usbd task while
			// loop() is stalled -- the only signal that survives a soft wedge.
			// Scan every read for it, whichever frame we were actually after.
			for (let w = 0; w + 4 < d.length; w++) {
				if (d[w] === MARK_WEDGE && d[w + 1] === 3) {
					this.events.onWedge({
						stage: d[w + 2],
						stallMs: d[w + 3] | (d[w + 4] << 8),
					});
					break;
				}
			}

			let i = 0;
			while (i < d.length && d[i] !== marker) i++;
			if (i + 2 > d.length) return null;
			const len = d[i + 1];
			// A transfer that ended mid-frame must be dropped whole: applying a
			// half blob silently skips every late field (reset cause, stack
			// stats) and reads as though the puck reported nothing wrong.
			if (i + 2 + len > d.length) return null;
			const p = d.slice(i + 2, i + 2 + len);
			return p.length >= minLen ? p : null;
		} catch (e) {
			if (this.dev) this.log.error(`read err: ${(e as Error).message}`);
			return null;
		}
	}

	readBlob() {
		return this.readFrame(MARK_STATUS, 12);
	}

	/**
	 * One raw read, for the streams that accumulate their own frames across
	 * many transfers (capture ring, flight recorder) rather than expecting one
	 * complete frame per read.
	 */
	async readRaw(len: number): Promise<Uint8Array | null> {
		if (!this.dev) return null;
		const r = await this.dev.transferIn(this.epIn, len);
		if (r.status !== 'ok' || !r.data) return null;
		return new Uint8Array(r.data.buffer);
	}

	// ---- firmware update ----
	//
	// One persistent transferIn at a time. A wait that times out leaves its
	// read PENDING for the next wait; issuing a second concurrent read would
	// silently eat the data the first one eventually resolves with.
	private fwupRead: Promise<USBInTransferResult> | null = null;

	/** Drop any read pending against a previous session or device. */
	resetFwupRead() {
		this.fwupRead = null;
	}

	async fwupSend(cmd: number[]): Promise<void> {
		if (!this.dev) throw new Error('device gone');
		await this.dev.transferOut(this.epOut, new Uint8Array(cmd));
	}

	async fwupAckWait(timeoutMs: number): Promise<FwupAck | null> {
		const deadline = Date.now() + timeoutMs;
		for (;;) {
			const remain = deadline - Date.now();
			if (remain <= 0) return null;
			if (!this.dev) throw new Error('device gone');
			if (!this.fwupRead) this.fwupRead = this.dev.transferIn(this.epIn, 64);
			const r = await Promise.race([
				this.fwupRead,
				new Promise<'timeout'>((res) => setTimeout(() => res('timeout'), remain)),
			]);
			if (r === 'timeout') return null;
			this.fwupRead = null;
			if (r.status !== 'ok' || !r.data) continue;
			const ack = parseAck(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength));
			if (ack) return ack;
		}
	}

	/** Send a control op (all idempotent firmware-side) and wait, resending on timeout. */
	async fwupCtl(cmd: number[], timeoutMs: number, label: string): Promise<FwupAck> {
		for (let t = 0; t < 3; t++) {
			await this.fwupSend(cmd);
			const a = await this.fwupAckWait(timeoutMs);
			if (a) return a;
		}
		throw new Error(`no response to ${label}`);
	}

	/**
	 * Read a [0xAA][count][count*16] lizard frame, accumulating packets until
	 * it is complete. Unlike readFrame this cannot use a single generous read:
	 * a full 32-binding map is 514 bytes, well over one transfer.
	 */
	async readLizardFrame(): Promise<{ acc: Uint8Array; count: number } | null> {
		if (!this.dev) return null;
		let acc = new Uint8Array(0);
		for (let guard = 0; guard < 64; guard++) {
			const r = await this.dev.transferIn(this.epIn, 128);
			if (r.status !== 'ok' || !r.data) break;
			const d = new Uint8Array(r.data.buffer);
			const m = new Uint8Array(acc.length + d.length);
			m.set(acc);
			m.set(d, acc.length);
			acc = m;
			let i = 0;
			while (i < acc.length && acc[i] !== MARK_LIZARD) i++;
			if (i > 0) acc = acc.slice(i);
			if (acc.length < 2) continue;
			const count = acc[1];
			if (acc.length >= 2 + count * 16) return { acc, count };
		}
		return null;
	}
}
