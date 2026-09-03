// Diagnostics: the capture ring, the flight recorder, the hang log and the
// stability test. These share the device endpoint, so each sets device.busy to
// suspend the status poll while it owns the pipe.

import { CAP_OP, decodeCaptureFrames, formatCapture, type CaptureEntry } from '$lib/protocol/capture';
import { decodeFlightFrames, FLIGHT_OP, type FlightEvent, type FlightHeader } from '$lib/protocol/flight';
import { OP } from '$lib/protocol/fields';
import { logs } from '$lib/state/log.svelte';
import { trail } from '$lib/state/trail.svelte';

export interface HangRow {
	time: string;
	/** Seconds the puck stayed up before this reset, when a stability run timed it. */
	uptime: number | null;
	reason: string;
	stage: string;
	pc: string;
	lr: string;
	usbd: number | null;
}

/** Seconds as "1m 12s", for uptimes shown next to a reset. */
export function fmtDuration(s: number): string {
	const r = Math.round(s);
	return r >= 60 ? `${Math.floor(r / 60)}m ${r % 60}s` : `${r}s`;
}

/**
 * What diagnostics needs from the device, declared rather than imported. The
 * device store owns the hang log and calls into here on every reset, so
 * importing it back would make the two modules circular.
 */
export interface DevicePort {
	readonly connected: boolean;
	busy: { capture: boolean; flight: boolean };
	sendRaw: (bytes: number[]) => Promise<void>;
	readRaw: (len: number) => Promise<Uint8Array | null>;
}

class DiagState {
	/** Set once by the device store at init. */
	private dev: DevicePort | null = null;

	bind(dev: DevicePort) {
		this.dev = dev;
	}

	private get port(): DevicePort {
		if (!this.dev) throw new Error('diagnostics used before the device store bound itself');
		return this.dev;
	}

	// ---- capture ----
	capturing = $state(false);
	capLines = $state<CaptureEntry[]>([]);
	private liveTimer: ReturnType<typeof setInterval> | null = null;
	private draining = false;

	get captureText() {
		return formatCapture(this.capLines);
	}

	async startCapture() {
		if (!this.port.connected || this.capturing) return;
		this.capturing = true;
		this.capLines = [];
		this.port.busy.capture = true;
		await this.port.sendRaw([CAP_OP.arm, 1]);
		this.liveTimer = setInterval(() => void this.drain(), 100);
		logs.info('capture started — dumping the ring from boot, then live');
	}

	async stopCapture() {
		if (this.liveTimer) {
			clearInterval(this.liveTimer);
			this.liveTimer = null;
		}
		await this.drain();
		try {
			await this.port.sendRaw([CAP_OP.arm, 0]);
		} catch {
			// Disarming is best-effort; the ring stops mattering once we stop reading.
		}
		this.capturing = false;
		this.port.busy.capture = false;
		logs.info(`capture stopped — ${this.capLines.length} entries`);
	}

	private async drain() {
		if (!this.port.connected || this.draining) return;
		this.draining = true;
		try {
			await this.port.sendRaw([CAP_OP.drain]);
			let acc = new Uint8Array(0);
			let done = false;
			// Guard the loop: a firmware that never sends the end frame must not
			// spin here forever holding the endpoint.
			for (let guard = 0; !done && guard < 400; guard++) {
				const d = await this.port.readRaw(64);
				if (!d) break;
				const m = new Uint8Array(acc.length + d.length);
				m.set(acc);
				m.set(d, acc.length);
				acc = m;
				const r = decodeCaptureFrames(acc);
				if (r.entries.length) this.capLines.push(...r.entries);
				acc = acc.slice(r.consumed);
				done = r.done;
			}
		} catch (e) {
			logs.error(`drain err: ${(e as Error).message}`);
		} finally {
			this.draining = false;
		}
	}

	// ---- flight recorder ----
	flightHeader = $state<FlightHeader | null>(null);
	flightEvents = $state<FlightEvent[]>([]);
	flightLoading = $state(false);

	async loadFlight() {
		if (!this.port.connected || this.port.busy.flight) return;
		this.port.busy.flight = true;
		this.flightLoading = true;
		this.flightHeader = null;
		this.flightEvents = [];
		try {
			let restart = true;
			let done = false;
			let acc = new Uint8Array(0);
			for (let guard = 0; !done && guard < 600; guard++) {
				// The stream is pumped: 0x10 with restart, then 0x10 to continue,
				// accumulating across FIFO-sized batches until the end frame.
				await this.port.sendRaw([FLIGHT_OP, restart ? 1 : 0]);
				restart = false;
				const d = await this.port.readRaw(192);
				if (!d) break;
				const m = new Uint8Array(acc.length + d.length);
				m.set(acc);
				m.set(d, acc.length);
				acc = m;
				const r = decodeFlightFrames(acc);
				if (r.header) this.flightHeader = r.header;
				if (r.events.length) this.flightEvents.push(...r.events);
				acc = acc.slice(r.consumed);
				done = r.done;
			}
		} catch (e) {
			logs.error(`flight err: ${(e as Error).message}`);
		} finally {
			this.flightLoading = false;
			this.port.busy.flight = false;
		}
	}

	// ---- hang log ----
	// One row per reset, accumulated in this tab. A page refresh clears it, so
	// the loop-state trail (localStorage) is the durable record.
	hangLog = $state<HangRow[]>([]);

	addHang(row: HangRow) {
		this.hangLog.unshift(row);
	}

	clearHangLog() {
		this.hangLog = [];
		logs.info('hang log cleared');
	}

	get hangCsv() {
		const head = 'time,uptime_s,reason,stage,pc,lr,usbd_words';
		const rows = this.hangLog.map((r) =>
			[r.time, r.uptime ?? '', r.reason, r.stage, r.pc, r.lr, r.usbd ?? ''].join(','),
		);
		return [head, ...rows].join('\n');
	}

	// ---- stability test ----
	// Buzzes the controllers every 10s to keep them awake and times how long the
	// puck stays up. Kept in this tab: a refresh clears the count.
	stabArmed = $state(false);
	stabStart = 0;
	stabRuns = $state<number[]>([]);
	private stabTimer: ReturnType<typeof setInterval> | null = null;

	async toggleStability() {
		if (this.stabArmed) {
			this.stabArmed = false;
			if (this.stabTimer) clearInterval(this.stabTimer);
			this.stabTimer = null;
			this.stabStart = 0;
			logs.info('stability test stopped');
			return;
		}
		this.stabArmed = true;
		this.stabStart = Date.now();
		await this.port.sendRaw([OP.stabilityBuzz, 1]);
		this.stabTimer = setInterval(() => void this.port.sendRaw([OP.stabilityBuzz, 1]), 10_000);
		logs.info('stability test started — buzzing every 10 s, timing uptime');
	}

	/** Called on disconnect: this is the reset that ended the run. */
	noteReset(): number | null {
		if (!this.stabArmed || !this.stabStart) return null;
		const secs = (Date.now() - this.stabStart) / 1000;
		this.stabRuns.unshift(secs);
		this.stabStart = Date.now();
		logs.warn(`stability: stayed up ${fmtDuration(secs)}, then reset`);
		trail.add(`stability run ended after ${fmtDuration(secs)}`);
		return secs;
	}
}

/** Capture ring, flight recorder, hang log and the stability test. */
export const diag = new DiagState();
