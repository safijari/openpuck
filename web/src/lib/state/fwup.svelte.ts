// Firmware update runner + its blocking modal state. The modal is the only
// interactive surface while an update runs: it covers everything else so a
// stray click cannot change a mode or write a field mid-flash.

import { crc32, errText, FWUP_CHUNK, FWUP_OP, FWUP_STATUS_RESYNC, maxSends, u32le } from '$lib/protocol/firmware';
import type { Transport } from '$lib/usb/transport';
import { logs } from '$lib/state/log.svelte';

export interface ModalState {
	open: boolean;
	title: string;
	stage: string;
	/** null renders an indeterminate bar. */
	pct: number | null;
	detail: string;
	failed: boolean;
	done: boolean;
}

const IDLE: ModalState = {
	open: false,
	title: '',
	stage: '',
	pct: null,
	detail: '',
	failed: false,
	done: false,
};

class FwupState {
	modal = $state<ModalState>({ ...IDLE });

	open(title: string) {
		this.modal = { ...IDLE, open: true, title, stage: 'Preparing…' };
	}

	stage(stage: string, pct: number | null = null) {
		this.modal.stage = stage;
		this.modal.pct = pct;
	}

	finish(ok: boolean, msg: string, failTitle = 'Update failed') {
		this.modal.done = true;
		this.modal.failed = !ok;
		this.modal.stage = msg;
		this.modal.pct = ok ? 100 : this.modal.pct;
		if (!ok) this.modal.title = failTitle;
	}

	close() {
		this.modal = { ...IDLE };
	}

	/**
	 * Stream an image into the puck's spare flash, have the puck verify it, then
	 * reboot to apply. Nothing is armed until the on-device CRC passes.
	 */
	async run(transport: Transport, image: Uint8Array) {
		const kb = Math.round(image.length / 1024);
		transport.resetFwupRead();
		// Drain anything stale on the IN pipe: a leftover ack from an aborted
		// run, or a late status blob.
		while ((await transport.fwupAckWait(400)) !== null) {
			/* keep draining */
		}

		logs.info(
			`firmware update: staging ${kb} KiB into the puck's spare flash — input may stutter briefly during page erases`,
		);

		const begin = await transport.fwupCtl(
			[FWUP_OP.begin, ...u32le(image.length), ...u32le(crc32(image))],
			4000,
			'begin',
		);
		if (begin.status) throw new Error(`begin rejected: ${errText(begin.status)}`);

		// Send one chunk, then read acks until one shows progress. A timeout
		// resends (covers a truly lost command); a stale no-progress ack -- the
		// surplus twin of a resent chunk -- is read past WITHOUT resending, so
		// retries cannot snowball. The ack's nextOff is authoritative: the
		// firmware skips duplicate chunks and re-acks, flash words are never
		// written twice, so every path resynchronises here.
		let off = 0;
		let sends = 0;
		let lastShown = -1;
		const cap = maxSends(image.length);

		while (off < image.length) {
			const len = Math.min(FWUP_CHUNK, image.length - off);
			await transport.fwupSend([FWUP_OP.chunk, ...u32le(off), len, ...image.subarray(off, off + len)]);
			if (++sends > cap) throw new Error(`transfer not converging at offset ${off}`);

			for (let reads = 0; reads < 8; reads++) {
				const ack = await transport.fwupAckWait(2500);
				if (ack === null) break; // timeout: resend this chunk
				if (ack.status === FWUP_STATUS_RESYNC) {
					off = ack.off; // firmware says where it wants us
					break;
				}
				if (ack.status !== 0) throw new Error(`chunk rejected at offset ${off}: ${errText(ack.status)}`);
				if (ack.off > off) {
					off = ack.off;
					break;
				}
				// else: stale ack, keep reading -- the real one is behind it
			}

			const pct = Math.floor((off * 50) / image.length) * 2;
			if (pct !== lastShown) {
				lastShown = pct;
				this.stage('Sending to the puck', pct);
			}
		}

		this.stage('Verifying on the puck', null);
		const commit = await transport.fwupCtl([FWUP_OP.verifyCommit], 8000, 'verify+commit');
		if (commit.status) throw new Error(`verify+commit rejected: ${errText(commit.status)}`);
		logs.ok('image staged + CRC-verified on the puck — update armed');

		this.stage('Rebooting', null);
		try {
			await transport.fwupSend([FWUP_OP.reboot]);
		} catch {
			// The device drops mid-call; that is the point.
		}
		logs.info(
			'rebooting to apply: the puck goes dark ~5 s while the new firmware is written, ' +
				'then re-enumerates — the panel reconnects itself',
		);
	}
}

/** The firmware update runner and its blocking progress modal. */
export const fwup = new FwupState();
