<script lang="ts">
	import { device } from '$lib/state/device.svelte';
	import { hexId } from '$lib/protocol/dongle';
	import Panel from '$lib/components/Panel.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import ConfirmDialog, { type ConfirmSpec } from '$lib/components/ConfirmDialog.svelte';

	const d = $derived(device.dongle);
	let confirming = $state<ConfirmSpec | null>(null);

	function askRemove(slot: number, serial: string) {
		confirming = {
			title: `Remove paired puck "${serial || `slot ${slot}`}"?`,
			body: ['The ReversePuck forgets this pairing; re-pair with scpair.py to use it again.'],
			confirmLabel: 'Remove',
			danger: true,
			onConfirm: () => void device.removePairedPuck(slot),
		};
	}
</script>

<Panel title="Paired pucks">
	{#snippet info()}
		<InfoPopover title="ReversePuck">
			This is a <strong>ReversePuck</strong> controller dongle (28DE:1302). It emulates a Steam Controller to whichever
			puck it's paired with. Pair new pucks with <code>scpair.py</code>; use the
			<em>Firmware</em> section to flash a new dongle build.
		</InfoPopover>
	{/snippet}

	{#if !d}
		<p class="text-app-muted text-sm">Reading paired pucks…</p>
	{:else}
		<div class="mb-3 flex flex-wrap items-center gap-3">
			<span
				class="rounded-full px-2 py-0.5 text-xs font-semibold
				{d.linkUp ? 'bg-success-100-900 text-success-700-300' : 'bg-error-100-900 text-error-700-300'}"
			>
				{d.linkUp ? 'RF link up' : 'no RF link'}
			</span>
			<span class="text-app-muted text-xs">{d.forwarding ? 'forwarding a Steam Deck' : 'idle'}</span>
		</div>

		{#if !d.pucks.length}
			<p class="text-app-muted text-sm">
				No pucks paired yet. Pair one with <code>scpair.py</code> (see ReversePuck/README.md), then it appears here.
			</p>
		{:else}
			<div class="border-app-line divide-app-line-soft divide-y rounded-base border">
				{#each d.pucks as p (p.slot)}
					<div class="flex items-center gap-3 px-3 py-2">
						<span
							class="shrink-0 rounded-full px-2 py-0.5 text-[10px] font-semibold
							{p.alive ? 'bg-success-100-900 text-success-700-300' : 'bg-app-hover text-app-muted'}"
						>
							{p.alive ? 'live' : 'offline'}
						</span>
						<div class="min-w-0 flex-1">
							<div class="truncate text-sm font-semibold">{p.serial || '(unnamed puck)'}</div>
							<div class="text-app-muted truncate font-mono text-xs">
								slot {p.slot} · puuid {hexId(p.puuid)} · iuuid {hexId(p.iuuid)}
							</div>
						</div>
						<button
							type="button"
							class="btn preset-tonal-error btn-sm shrink-0"
							title="Un-bond this puck from the ReversePuck. You'll need to re-pair to use it again."
							onclick={() => askRemove(p.slot, p.serial)}
						>
							Remove
						</button>
					</div>
				{/each}
			</div>
		{/if}
	{/if}
</Panel>

<ConfirmDialog spec={confirming} onCancel={() => (confirming = null)} />
