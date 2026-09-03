<script lang="ts">
	import DownloadIcon from '@lucide/svelte/icons/download';
	import UploadIcon from '@lucide/svelte/icons/upload';
	import { device } from '$lib/state/device.svelte';
	import { bondedCount, parseBackup, type Backup } from '$lib/protocol/backup';
	import { logs } from '$lib/state/log.svelte';
	import ConfirmDialog, { type ConfirmSpec } from '$lib/components/ConfirmDialog.svelte';
	import PairedPucks from '$lib/sections/PairedPucks.svelte';
	import Panel from '$lib/components/Panel.svelte';
	import Stat from '$lib/components/Stat.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';

	const status = $derived(device.status);

	let fileInput = $state<HTMLInputElement | null>(null);
	let confirming = $state<ConfirmSpec | null>(null);

	async function pickBackup(f: File | undefined) {
		if (!f) return;
		let backup: Backup;
		try {
			backup = parseBackup(await f.text());
		} catch (e) {
			logs.error(`import: ${(e as Error).message}`);
			return;
		}
		// The panel only talks to the puck you last connected, and a restore is
		// irreversible on the target -- so name the serial being written to.
		confirming = {
			title: 'Restore onto the connected puck?',
			body: [
				`Target: serial ${device.serial || '?'}.`,
				'Make sure this is the TARGET puck, not the one you exported from.',
				`This OVERWRITES its ${bondedCount(backup)} controller pairing(s) and ALL settings, then reboots it.`,
				'Result: any controller paired to the original puck will connect to this one with no re-pairing.',
			],
			confirmLabel: 'Restore',
			danger: true,
			onConfirm: () => void device.importBackup(backup),
		};
	}
</script>

{#if device.isDongle}
	<div class="mb-4"><PairedPucks /></div>
{/if}

<div class="grid gap-4 autofit-[340px]">
	{#if !device.isDongle}
		<Panel title="Version">
			{#snippet info()}
				<InfoPopover title="Version">
					This section reflects the connected puck. Reconnect after flashing or changing modes to verify what is
					currently running.
				</InfoPopover>
			{/snippet}
			<div class="grid grid-cols-2 gap-2">
				<Stat label="Firmware build" value={status?.build.id} />
				<Stat label="Status protocol" value={status ? `v${status.protocol}` : null} />
				<Stat
					label="Git tree"
					value={status ? (status.build.dirty ? 'dirty' : 'clean') : null}
					tone={status ? (status.build.dirty ? 'warn' : 'up') : 'none'}
				/>
				<Stat
					label="Panel update"
					value={status ? (status.caps.panelUpdate ? 'supported' : 'manual UF2 only') : null}
					tone={status && !status.caps.panelUpdate ? 'warn' : 'none'}
				/>
			</div>
		</Panel>
	{/if}

	{#if !device.isDongle}
		<Panel title="Backup &amp; clone">
			{#snippet info()}
				<InfoPopover title="Backup &amp; clone">
					Save this puck's controller pairings <em>and</em> every setting to a file, then restore them onto another puck.
					The other puck becomes a clone — any controller paired to this one connects to it with no re-pairing. Import overwrites
					all pairings and settings on the connected puck, then reboots it.
				</InfoPopover>
			{/snippet}
			<input
				bind:this={fileInput}
				type="file"
				accept="application/json,.json"
				class="hidden"
				onchange={(e) => {
					pickBackup(e.currentTarget.files?.[0]);
					e.currentTarget.value = '';
				}}
			/>
			<div class="flex flex-wrap gap-2">
				<button
					type="button"
					class="btn preset-filled-primary-500 btn-sm flex items-center gap-1.5"
					disabled={!device.connected || device.busy.backup}
					onclick={() => device.exportBackup()}
				>
					<DownloadIcon size={14} /> Export to file…
				</button>
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
					disabled={!device.connected || device.busy.backup}
					onclick={() => fileInput?.click()}
				>
					<UploadIcon size={14} /> Import from file…
				</button>
			</div>
			<p class="text-app-muted mt-2 text-xs">
				Tip: pair every controller to one puck, export once, then import onto each spare.
			</p>
		</Panel>
	{/if}
</div>

<ConfirmDialog spec={confirming} onCancel={() => (confirming = null)} />
