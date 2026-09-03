// Sniffer session state. The read loop runs at the full capture rate, so the
// hot path only stores raw bytes: hex formatting and DOM work are deferred to a
// display-rate tick. A blocked read backs up the device FIFO, stalls its drain,
// and makes the on-device ring drop frames (the `drops` counter).

import {
	EMPTY_FILTER,
	ensureHex,
	formatTextExport,
	parseStream,
	passesFilter,
	SNIFFER_CMD,
	SNIFFER_VID,
	type FilterSpec,
	type SnifferFrame,
	type SnifferStatus,
} from '$lib/protocol/sniffer';
import { logs } from '$lib/state/log.svelte';

/** Frames retained for re-filtering. The capture record is separate and uncapped. */
const FRAMEBUF = 8000;
/** Rows rendered live; the full record is still exported. */
export const MAX_ROWS = 600;

class SnifferState {
	connected = $state(false);
	status = $state<SnifferStatus | null>(null);
	capturing = $state(false);
	surveying = $state(false);
	filter = $state<FilterSpec>({ ...EMPTY_FILTER });

	/** Counters, updated on the render tick rather than per frame. */
	stats = $state({ pc: 0, cp: 0, bad: 0, recorded: 0, shown: 0 });
	rows = $state<SnifferFrame[]>([]);

	private dev: USBDevice | null = null;
	private epIn = 0;
	private epOut = 0;
	private ifNum = 0;
	private reading = false;
	private acc = new Uint8Array(0);
	private all: SnifferFrame[] = [];
	private record: SnifferFrame[] = [];
	private seq = 0;
	private dirty = false;
	private raf = 0;

	async connect() {
		try {
			const d = await navigator.usb.requestDevice({ filters: [{ vendorId: SNIFFER_VID }] });
			this.dev = d;
			await d.open();
			if (d.configuration === null) await d.selectConfiguration(1);

			let found: { ifNum: number; epIn: number; epOut: number } | null = null;
			for (const itf of d.configuration!.interfaces) {
				const a = itf.alternate;
				if (a.interfaceClass !== 0xff || a.interfaceSubclass === 0x5d) continue;
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
				logs.error('no WebUSB bulk interface — is this the sniffer board (OpenPuck Sniffer)?');
				this.dev = null;
				return;
			}
			this.ifNum = found.ifNum;
			this.epIn = found.epIn;
			this.epOut = found.epOut;
			await d.claimInterface(this.ifNum);
			await d.controlTransferOut({
				requestType: 'class',
				recipient: 'interface',
				request: 0x22,
				value: 0x01,
				index: this.ifNum,
			});

			this.connected = true;
			logs.ok(`connected — iface ${this.ifNum} epIn ${this.epIn} epOut ${this.epOut} — waiting for stream…`);

			navigator.usb.addEventListener('disconnect', (e) => {
				if ((e as USBConnectionEvent).device === this.dev) this.onGone();
			});

			// Silence here means the board is running a firmware that does not
			// stream, which otherwise looks exactly like an idle radio.
			let sawData = false;
			const seen = () => (sawData = true);
			setTimeout(() => {
				if (this.dev && !sawData) {
					logs.warn(
						'connected but NO bytes from the sniffer in 2s — firmware not streaming ' +
							'(re-flash the updated sniffer build) or wrong interface.',
					);
				}
			}, 2000);

			this.reading = true;
			void this.readLoop(seen);
			this.raf = requestAnimationFrame(() => this.renderTick());
		} catch (e) {
			logs.error(`connect failed: ${(e as Error).message}`);
			this.dev = null;
		}
	}

	private onGone() {
		this.reading = false;
		this.dev = null;
		this.connected = false;
		this.capturing = false;
		cancelAnimationFrame(this.raf);
		logs.warn('sniffer disconnected');
	}

	async cmd(...bytes: number[]) {
		if (!this.dev) return;
		try {
			await this.dev.transferOut(this.epOut, new Uint8Array(bytes));
		} catch (e) {
			logs.error(`write err: ${(e as Error).message}`);
		}
	}

	private async readLoop(seen: () => void) {
		while (this.reading && this.dev) {
			try {
				const r = await this.dev.transferIn(this.epIn, 4096);
				if (r.status !== 'ok' || !r.data) continue;
				seen();
				const d = new Uint8Array(r.data.buffer);
				const m = new Uint8Array(this.acc.length + d.length);
				m.set(this.acc);
				m.set(d, this.acc.length);
				this.acc = m;

				const { status, frames, consumed } = parseStream(this.acc);
				this.acc = this.acc.slice(consumed);
				if (status) this.status = status;
				for (const f of frames) this.push(f as SnifferFrame);
			} catch {
				// A read error means the device went away; onGone handles it.
				break;
			}
		}
	}

	private push(f: SnifferFrame) {
		f.seq = ++this.seq;
		if (f.dir === 'P→C') this.stats.pc++;
		else if (f.dir === 'C→P') this.stats.cp++;
		if (!f.crc) this.stats.bad++;
		if (this.capturing) this.record.push(f);
		this.all.push(f);
		// Bulk trim: a per-frame shift would be O(n) at 500+ frames/s.
		if (this.all.length > FRAMEBUF * 2) this.all.splice(0, this.all.length - FRAMEBUF);
		this.dirty = true;
	}

	/** Re-render at display rate, not capture rate. */
	private renderTick() {
		if (this.dirty) {
			this.dirty = false;
			this.applyFilters();
			this.stats.recorded = this.record.length;
		}
		this.raf = requestAnimationFrame(() => this.renderTick());
	}

	applyFilters() {
		const out: SnifferFrame[] = [];
		for (let i = this.all.length - 1; i >= 0 && out.length < MAX_ROWS; i--) {
			if (passesFilter(this.all[i], this.filter)) out.push(ensureHex(this.all[i]));
		}
		this.rows = out;
		this.stats.shown = out.length;
	}

	toggleCapture() {
		this.capturing = !this.capturing;
		logs.info(this.capturing ? 'capture started' : `capture stopped — ${this.record.length} frames`);
	}

	clear() {
		this.record = [];
		this.all = [];
		this.rows = [];
		this.stats = { pc: 0, cp: 0, bad: 0, recorded: 0, shown: 0 };
		logs.info('cleared');
	}

	private download(name: string, text: string, type: string) {
		const a = document.createElement('a');
		a.href = URL.createObjectURL(new Blob([text], { type }));
		a.download = name;
		a.click();
		URL.revokeObjectURL(a.href);
	}

	downloadJson() {
		// Materialise the hex and drop the byte arrays so the file matches the
		// shape the old page produced.
		const out = this.record.map((e) => {
			const { bytes, ...rest } = ensureHex(e);
			void bytes;
			return rest;
		});
		this.download('puck-sniff.json', JSON.stringify(out, null, 1), 'application/json');
	}

	downloadText() {
		this.download('puck-sniff.txt', formatTextExport(this.record), 'text/plain');
	}

	reacquire() {
		void this.cmd(SNIFFER_CMD.reacquire);
	}
	forgetBonds() {
		void this.cmd(SNIFFER_CMD.forgetBonds);
	}
	toggleSurvey() {
		this.surveying = !this.surveying;
		void this.cmd(SNIFFER_CMD.survey, this.surveying ? 1 : 0);
	}
	setTargetIbex(v: number) {
		void this.cmd(SNIFFER_CMD.targetIbex, v & 0xff);
	}
	pinChannel(ch: number) {
		void this.cmd(SNIFFER_CMD.pinChannel, ch & 0xff);
	}
	pinSession(vals: number[]) {
		void this.cmd(SNIFFER_CMD.pinSession, ...vals);
	}
}

/** The RF sniffer session: its own board, its own connection. */
export const sniffer = new SnifferState();
