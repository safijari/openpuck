import { parseBlob, STAGE_NAMES, type DeviceStatus } from '$lib/protocol/blob';
import { buildBlob, buildDongleFrame, FIXTURE_LIZARD } from '$lib/protocol/fixtures';
import { Transport, MARK_BONDS, MARK_PAIRED } from '$lib/usb/transport';
import {
	BK_OP,
	BOND_FRAME_MIN,
	backupFilename,
	bondRecord,
	buildBackup,
	MODE_UNCHANGED,
	parseBackup,
	restoreSteps,
	type Backup,
} from '$lib/protocol/backup';
import { FIELD, padStickField, typeField, TYPE_OFF } from '$lib/protocol/fields';
import { fwup } from '$lib/state/fwup.svelte';
import { decodeBindings, encodeBinding, filterSavable, LZ_MAX, LZ_OP, type LizardBinding } from '$lib/protocol/lizard';
import { logs } from '$lib/state/log.svelte';
import { trail } from '$lib/state/trail.svelte';
import { diag } from '$lib/state/diag.svelte';
import { DONGLE_OP, parseDongleStatus, type DongleStatus } from '$lib/protocol/dongle';

export type ConnState = 'disconnected' | 'connecting' | 'connected' | 'lost';

/** Status poll cadence, matching the original panel. */
const POLL_MS = 600;
/**
 * A soft wedge still lets the usbd task answer, so the STALLED pill covers it.
 * The lockup/hardfault class (issue #72) kills the whole MCU: USB goes silent,
 * transferIn just pends, and the panel would sit there showing "running"
 * forever. Only the host can see that, hence this timeout.
 */
const HEARTBEAT_TIMEOUT_MS = 2500;

class DeviceState {
	conn = $state<ConnState>('disconnected');
	status = $state<DeviceStatus | null>(null);
	/** True for a ReversePuck controller dongle (28DE:1302). */
	isDongle = $state(false);
	/** Populated instead of `status` while a dongle is connected. */
	dongle = $state<DongleStatus | null>(null);
	serial = $state('');
	activeSlot = $state(0);
	/** Set when no status blob has arrived for longer than the timeout. */
	hardWedge = $state<number | null>(null);

	/** Raw last payload, kept for the backup export which replays every field. */
	lastBlob: Uint8Array | null = null;

	private transport: Transport;
	private polling = false;
	private inflight = false;
	private lastBlobTs = 0;
	private hbLostEpisode = false;
	private stallEpisode = false;
	private stallPeak = 0;
	/**
	 * A reset queued at disconnect. Its reason, PC and stack come from the first
	 * blob after reconnect, so the row is only complete once the puck is back.
	 */
	private pendingHang: { t: number; uptimeSecs: number | null } | null = null;
	/** Auto-load the flight trail once per connection, after a fault-class boot. */
	private flightAutoLoaded = false;

	/**
	 * Paths that legitimately take over the endpoint and must suspend the blob
	 * poll: a nested transferIn would steal the other path's reply.
	 */
	busy = $state({
		capture: false,
		backup: false,
		flight: false,
		lizard: false,
	});

	constructor() {
		this.transport = new Transport(
			{
				onOpen: ({ isDongle, serial }) => {
					this.isDongle = isDongle;
					this.dongle = null;
					this.serial = serial;
					this.conn = 'connected';
					this.hardWedge = null;
					this.lastBlobTs = 0;
					this.hbLostEpisode = false;
					this.lizardLoaded = false;
					void this.startPolling();
				},
				onGone: () => {
					this.polling = false;
					this.conn = 'lost';
					this.status = null;
					this.lastBlobTs = 0;
					this.hbLostEpisode = false;
					this.hardWedge = null;
					logs.warn('device disconnected (mode-switch / watchdog reboot) — auto-reconnecting when it returns');
					trail.add('device disconnected — reset or replug; reason logged on reconnect');
				},
				onWedge: ({ stage, stallMs }) => {
					const name = STAGE_NAMES[stage] ?? `stage ${stage}`;
					trail.add(`WEDGED @ ${name} (${stallMs}ms — live 0xA9 report; watchdog reset imminent)`);
					logs.error(`LOOP WEDGED @ ${name} (stuck ${stallMs}ms) — watchdog reset imminent`);
				},
			},
			logs,
		);
	}

	get connected() {
		return this.conn === 'connected';
	}

	/** For the firmware runner, which needs the raw endpoint discipline. */
	get transportRef() {
		return this.transport;
	}

	get caps() {
		return this.status?.caps ?? null;
	}

	get bondedSlots(): number[] {
		const s = this.status;
		if (!s) return [];
		return s.slots.map((slot, i) => (slot ? i : -1)).filter((i) => i >= 0);
	}

	get anyBusy() {
		return this.busy.capture || this.busy.backup || this.busy.flight || this.busy.lizard;
	}

	init() {
		diag.bind({
			get connected() {
				return device.connected;
			},
			busy: this.busy,
			sendRaw: (b) => this.sendRaw(b),
			readRaw: (n) => this.transport.readRaw(n),
		});
		this.transport.listen();
		void this.transport.autoConnect();
		setInterval(() => this.checkHeartbeat(), 1000);
	}

	async connect() {
		this.conn = 'connecting';
		const ok = await this.transport.connect();
		if (!ok && this.conn === 'connecting') this.conn = 'disconnected';
	}

	/** Write one config field, then re-read so the UI reflects what stuck. */
	/** Un-bond a paired puck from the dongle. */
	async removePairedPuck(slot: number) {
		await this.transport.send([DONGLE_OP.removePuck, slot & 0xff]);
		logs.info(`removed paired puck slot ${slot}`);
		// The firmware replies with a fresh 0xAC; the next poll refreshes the list.
	}

	async setField(field: number, value: number) {
		await this.transport.send([0x02, field, value & 0xff]);
		const p = await this.transport.readBlob();
		if (p) this.applyBlob(p);
	}

	async sendRaw(bytes: number[]) {
		await this.transport.send(bytes);
	}

	private applyBlob(p: Uint8Array) {
		this.lastBlob = p;
		this.lastBlobTs = Date.now();
		this.hardWedge = null;
		const next = parseBlob(p);

		// Log a stall once per episode, then again each time it climbs
		// meaningfully towards the ~8s watchdog, so the record survives the
		// reset that is probably coming.
		if (next.loop.stalled) {
			if (!this.stallEpisode || next.loop.stallMs > this.stallPeak + 800) {
				if (!this.stallEpisode) {
					trail.add(`STALLED @ ${next.loop.stage} (${next.loop.stallMs}ms, soft wedge — usbd still alive)`);
				}
				this.stallEpisode = true;
				this.stallPeak = next.loop.stallMs;
				logs.warn(`LOOP STALL @ ${next.loop.stage} (${next.loop.stallMs}ms) — watchdog reset imminent`);
			}
		} else if (this.stallEpisode) {
			trail.add(`running again — stall recovered without a reset (peaked ${this.stallPeak}ms)`);
			this.stallEpisode = false;
			this.stallPeak = 0;
		}

		// Clamp the selected slot to one that is actually bonded.
		if (!next.slots[this.activeSlot]) {
			const first = next.slots.findIndex((s) => s);
			this.activeSlot = first >= 0 ? first : 0;
		}
		this.status = next;

		// Lazy, capability-gated: only after a blob proves the puck speaks v16+.
		if (!this.lizardLoaded && next.caps.lizard) {
			this.lizardLoaded = true;
			void this.loadLizard();
		}

		const hx = (v: number) => '0x' + v.toString(16).padStart(8, '0');
		const r = next.reset;

		// Complete the row queued at disconnect, now that this boot's reason,
		// stuck stage and captured PC have arrived.
		if (this.pendingHang && r) {
			diag.addHang({
				time: new Date(this.pendingHang.t).toLocaleTimeString(),
				uptime: this.pendingHang.uptimeSecs,
				reason: r.name,
				stage: r.hangStageName ?? '',
				pc: r.hangPC ? hx(r.hangPC) : '',
				lr: r.hangLR ? hx(r.hangLR) : '',
				usbd: next.loop.usbdStackWords,
			});
			trail.add(
				`reconnected — last reset: ${r.name}` +
					(r.hangStageName ? ` @ ${r.hangStageName}` : '') +
					(r.hangPC ? ` PC=${hx(r.hangPC)}` : ''),
			);
			this.pendingHang = null;
		}

		// The flight trail is the post-mortem of a real crash, so pull it once
		// per connection when the last boot was one. Deferred out of this call so
		// it never nests a transferIn inside the poll.
		if (r?.isFault && !this.flightAutoLoaded) {
			this.flightAutoLoaded = true;
			setTimeout(() => void diag.loadFlight(), 0);
		}
	}

	private async refresh() {
		if (!this.transport.connected || this.inflight || this.anyBusy) return;
		this.inflight = true;
		// try/finally matters: an exception anywhere in here must never leave
		// inflight latched true, which would kill every future poll silently and
		// read as a fake heartbeat loss.
		try {
			await this.transport.send([0x01]);
			if (this.isDongle) {
				const p = await this.transport.readFrame(MARK_PAIRED, 3, 256);
				if (p) {
					this.lastBlob = p;
					this.lastBlobTs = Date.now();
					this.hardWedge = null;
					// A dongle reconnect closes out the row queued on disconnect;
					// it reports no reset cause to classify it with.
					this.pendingHang = null;
					this.dongle = parseDongleStatus(p);
				}
			} else {
				const p = await this.transport.readBlob();
				if (p) this.applyBlob(p);
			}
		} catch (e) {
			logs.error(`refresh err: ${(e as Error).message}`);
		} finally {
			this.inflight = false;
		}
	}

	private async startPolling() {
		this.polling = true;
		await this.refresh();
		while (this.polling && this.transport.connected) {
			await new Promise((r) => setTimeout(r, POLL_MS));
			await this.refresh();
		}
	}

	private checkHeartbeat() {
		if (!this.transport.connected || !this.lastBlobTs || this.anyBusy) return;
		const age = Date.now() - this.lastBlobTs;
		if (age > HEARTBEAT_TIMEOUT_MS) {
			const secs = Math.round(age / 1000);
			this.hardWedge = secs;
			if (!this.hbLostEpisode) {
				this.hbLostEpisode = true;
				logs.error(
					`heartbeat lost — no status blob for ${secs}s: hard wedge ` +
						`(USB stack dead too, so the live wedge reporter can't run) — check the flight trail after the reset`,
				);
				trail.add(`HEARTBEAT LOST — no status for ${secs}s (hard wedge: USB silent, whole MCU likely stopped)`);
			}
		} else if (this.hbLostEpisode) {
			this.hbLostEpisode = false;
			this.hardWedge = null;
			trail.add('heartbeat back — status blobs resumed without a reset');
		}
	}

	// ---- lizard map ----
	lizard = $state<LizardBinding[]>([]);
	lizardStatus = $state('');
	/** One-shot per connection: the map is fetched lazily once v16+ is proven. */
	private lizardLoaded = false;

	/**
	 * Every lizard op sends 0x11..0x15 then does a blocking read. Pre-v16
	 * firmware drops the byte and never replies, so the read would hang the
	 * shared endpoint forever and take the status poll with it. Never send one
	 * until a status blob has proven the puck speaks v16.
	 */
	get lizardCapable() {
		return !!this.status?.caps.lizard;
	}

	private async lizardExchange<T>(fn: () => Promise<T>): Promise<T | null> {
		this.busy.lizard = true;
		// Let an in-flight status poll finish first; a nested transferIn would
		// steal its reply.
		for (let i = 0; i < 25 && this.inflight; i++) await new Promise((r) => setTimeout(r, 20));
		try {
			return await fn();
		} catch (e) {
			logs.error(`lizard err: ${(e as Error).message}`);
			return null;
		} finally {
			this.busy.lizard = false;
		}
	}

	private async readLizard(): Promise<LizardBinding[] | null> {
		const frame = await this.transport.readLizardFrame();
		return frame ? decodeBindings(frame.acc, frame.count) : null;
	}

	async loadLizard() {
		if (!this.lizardCapable) return;
		await this.lizardExchange(async () => {
			await this.transport.send([LZ_OP.dump]);
			const b = await this.readLizard();
			if (b) {
				this.lizard = b;
				logs.info(`lizard map loaded — ${b.length} bindings`);
			}
		});
	}

	async saveLizard() {
		if (!this.transport.connected || !this.lizardCapable) return;
		const valid = filterSavable($state.snapshot(this.lizard) as LizardBinding[]);
		const dropped = this.lizard.length - valid.length;
		await this.lizardExchange(async () => {
			await this.transport.send([LZ_OP.beginEdit, valid.length & 0xff]);
			for (let i = 0; i < valid.length; i++) await this.transport.send(encodeBinding(i, valid[i]));
			await this.transport.send([LZ_OP.commit]);
			const b = await this.readLizard();
			if (b) this.lizard = b;
			const skipped = dropped ? ` (skipped ${dropped} incomplete: no trigger/hold)` : '';
			logs.ok(`lizard map saved — ${valid.length} bindings${skipped}`);
		});
	}

	async resetLizard() {
		if (!this.transport.connected || !this.lizardCapable) return;
		await this.lizardExchange(async () => {
			await this.transport.send([LZ_OP.reset]);
			const b = await this.readLizard();
			if (b) this.lizard = b;
			logs.ok('lizard map reset to defaults');
		});
	}

	// ---- backup / clone ----

	/** Let an in-flight poll settle before driving the shared endpoints directly. */
	private async waitIdle() {
		for (let i = 0; i < 60 && this.inflight; i++) await new Promise((r) => setTimeout(r, 5));
	}

	private async readBonds(): Promise<Uint8Array | null> {
		await this.transport.send([BK_OP.dumpBonds]);
		for (let t = 0; t < 8; t++) {
			const bp = await this.transport.readFrame(MARK_BONDS, BOND_FRAME_MIN);
			if (bp) return bp;
		}
		return null;
	}

	async exportBackup() {
		if (!this.transport.connected) {
			logs.warn('not connected');
			return;
		}
		this.busy.backup = true;
		await this.waitIdle();
		try {
			// Take a fresh config snapshot rather than trusting the last poll.
			await this.transport.send([0x01]);
			const cfg = await this.transport.readBlob();
			if (cfg) this.lastBlob = cfg;
			if (!this.lastBlob) {
				logs.error('export: no config snapshot yet — wait a second and retry');
				return;
			}
			const bp = await this.readBonds();
			if (!bp) {
				logs.error('export: no bond data (firmware too old for 0x09 export — reflash)');
				return;
			}
			// Only snapshot the lizard map once its lazy load has finished; an
			// empty one would clear the target's bindings on restore.
			const map = this.lizardLoaded && this.lizardCapable && !this.busy.lizard ? this.lizard : null;
			const backup = buildBackup(this.lastBlob, bp, map ? ($state.snapshot(map) as typeof map) : null);

			const a = document.createElement('a');
			a.href = URL.createObjectURL(new Blob([JSON.stringify(backup, null, 2)], { type: 'application/json' }));
			a.download = backupFilename();
			a.click();
			URL.revokeObjectURL(a.href);
			logs.ok(`exported backup — ${backup.bonds.filter((b) => b.used).length} bonded slot(s) + all settings`);
		} finally {
			this.busy.backup = false;
		}
	}

	async importBackup(backup: Backup) {
		if (!this.transport.connected) {
			logs.warn('not connected');
			return;
		}
		this.busy.backup = true;
		await this.waitIdle();
		fwup.open('Restore backup');
		const c = backup.config;
		const total = restoreSteps(c, this.lizardCapable);
		let step = 0;
		let stage = '';
		const tick = () => fwup.stage(stage, Math.min(99, Math.floor((step * 100) / total)));

		try {
			// The #1 import failure is being connected to a puck still on old
			// firmware: it silently ignores the bond-import command below. Probe
			// for the 0xA7 answer first and fail loudly instead.
			fwup.stage('Checking puck firmware', null);
			if (!(await this.readBonds())) {
				logs.error('import ABORTED — the connected puck does not support import (old firmware).');
				fwup.finish(
					false,
					"This puck's firmware is too old to import a backup. Flash it with the latest OpenPuck build " +
						'(the same one that produced the export), reconnect, then import again.',
					'Import failed',
				);
				return;
			}

			// Every step is write-then-read, the same shape as the proven 0x02
			// sliders: the firmware acks with a status blob, and reading it keeps
			// the OUT pipe flowing and paces the writes.
			const w = async (bytes: number[]) => {
				await this.transport.send(bytes);
				await this.transport.readBlob();
				step++;
				tick();
			};
			const sf = (f: number, v: number) => w([0x02, f, (v | 0) & 0xff]);

			stage = 'Restoring settings';
			tick();
			await sf(FIELD.mouseDiv, c.mDiv);
			await sf(FIELD.mouseFriction, c.mFric);
			await sf(FIELD.persistMode, c.persistMode ? 1 : 0);
			for (let i = 0; i < 3; i++) await sf(FIELD.chordFace[i], c.chord[i]);
			// Absent in backups taken before these settings existed -- skip them
			// rather than replaying a default over the target's real value.
			if (Array.isArray(c.chordD)) for (let i = 0; i < 4; i++) await sf(FIELD.chordDpad[i], c.chordD[i]);
			if (c.swGyroLegacy !== undefined) await sf(FIELD.swGyroMap, c.swGyroLegacy ? 1 : 0);

			const typeN = Array.isArray(c.types) ? Math.min(c.types.length, 4) : 0;
			for (let et = 0; et < typeN; et++) {
				const t = c.types[et];
				for (let k = 0; k < 4; k++) await sf(typeField(et, TYPE_OFF.back + k), t.back[k]);
				await sf(typeField(et, TYPE_OFF.qam), t.qam);
				await sf(typeField(et, TYPE_OFF.abSwap), t.abSwap ? 1 : 0);
				await sf(typeField(et, TYPE_OFF.padHaptics), t.pad ? 1 : 0);
				await sf(typeField(et, TYPE_OFF.led), t.led);
				await sf(typeField(et, TYPE_OFF.rumble), t.rumble !== undefined ? t.rumble : 1);
				if (Array.isArray(t.padStick)) {
					await sf(padStickField(et, 0), t.padStick[0]);
					await sf(padStickField(et, 1), t.padStick[1]);
				}
			}
			logs.info(`settings replayed${typeN ? '' : ' (no per-type config in this backup)'}`);

			if (Array.isArray(c.lizardMap) && this.lizardCapable) {
				stage = 'Restoring button map';
				tick();
				const map = c.lizardMap.slice(0, LZ_MAX);
				await this.transport.send([LZ_OP.beginEdit, map.length & 0xff]);
				for (let i = 0; i < map.length; i++) {
					await this.transport.send(encodeBinding(i, map[i]));
					step++;
					tick();
				}
				await this.transport.send([LZ_OP.commit]);
				// Drain the 0xAA echo so the pipe is clean for the bond writes.
				await this.transport.readLizardFrame();
				logs.ok(`lizard map restored — ${map.length} bindings`);
			}

			stage = 'Restoring controller pairings';
			tick();
			let n = 0;
			for (let s = 0; s < 4; s++) {
				const { used, rec } = bondRecord(backup.bonds.find((x) => x.slot === s));
				if (used) n++;
				await w([BK_OP.writeBond, s, used, ...rec]);
			}
			logs.info(`wrote ${n} bond slot(s) — committing + rebooting`);

			// Commit persists the bonds, applies the mode and reboots, so there
			// is no ack to wait for.
			stage = 'Committing + rebooting';
			step++;
			tick();
			await this.transport.send([BK_OP.commitReboot, (c.mode !== undefined ? c.mode : MODE_UNCHANGED) & 0xff]);
			logs.ok('import sent — puck is cloning and rebooting. Reconnect after it returns.');
			fwup.finish(true, 'Backup restored — the puck is cloning and rebooting. Reconnect after it returns.');
		} catch (e) {
			const msg = (e as Error).message;
			logs.error(`import FAILED — ${msg}`);
			fwup.finish(false, `${msg} — the import stopped partway; reconnect and retry.`, 'Import failed');
		} finally {
			this.busy.backup = false;
		}
	}

	/**
	 * True when the readings on screen came from a recorded blob rather than a
	 * puck. The UI must say so: everything else about this state is identical to
	 * a live connection, so without a marker it reads as real hardware.
	 */
	demo = $state(false);

	/** Renders the shell against a recorded blob, for layout work without hardware. */
	loadFixture() {
		// Fixture mode skips init(), so diagnostics still needs its port.
		diag.bind({
			get connected() {
				return device.connected;
			},
			busy: this.busy,
			sendRaw: (b) => this.sendRaw(b),
			readRaw: (n) => this.transport.readRaw(n),
		});
		this.demo = true;
		this.applyBlob(buildBlob());
		this.conn = 'connected';
		this.lizardLoaded = true; // no device to dump from
		this.lizard = structuredClone(FIXTURE_LIZARD);
	}

	/** Renders the reduced ReversePuck layout without a dongle attached. */
	loadDongleFixture() {
		diag.bind({
			get connected() {
				return device.connected;
			},
			busy: this.busy,
			sendRaw: (b) => this.sendRaw(b),
			readRaw: (n) => this.transport.readRaw(n),
		});
		this.demo = true;
		this.conn = 'connected';
		this.isDongle = true;
		// A dongle answers 0xAC only; it has no status blob, so `status` stays
		// null exactly as it would with real hardware.
		this.dongle = parseDongleStatus(buildDongleFrame());
	}
}

/** The connection, the 600ms status poll, and every write to the puck. */
export const device = new DeviceState();
/** Whether this browser implements WebUSB at all (Chrome and Edge do). */
export const supported = typeof navigator !== 'undefined' && 'usb' in navigator;
