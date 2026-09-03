<script lang="ts">
	import Trash2Icon from '@lucide/svelte/icons/trash-2';
	import { trail } from '$lib/state/trail.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import StreamPanel from './StreamPanel.svelte';
</script>

<StreamPanel id="trail" title="Loop trail" badge={`${trail.entries.length}`}>
	{#snippet toolbar()}
		<span class="text-app-muted flex items-center gap-1.5 text-xs">
			Every loop-state change, timestamped.
			<InfoPopover title="Loop-state trail">
				Stall episodes, live wedge reports, heartbeat loss (hard wedge, USB silent), recoveries, disconnects and the
				reset cause on reconnect — so an unattended hang is recorded even if you weren't watching. Saved in this browser
				(localStorage): it <strong>survives page refreshes</strong> and device reboots, and clears only with the button.
			</InfoPopover>
		</span>
		<button
			type="button"
			class="text-app-muted hover:text-error-700-300 ml-auto px-1"
			title="Clear trail"
			aria-label="Clear trail"
			onclick={() => trail.clear()}
		>
			<Trash2Icon size={14} />
		</button>
	{/snippet}

	{#if !trail.entries.length}
		<p class="text-app-muted">No events yet.</p>
	{:else}
		{#each trail.entries as e (e.t + e.msg)}
			<div class="flex gap-2">
				<span class="text-app-faint tabular-nums shrink-0">{new Date(e.t).toLocaleString()}</span>
				<span class="break-all whitespace-pre-wrap">{e.msg}</span>
			</div>
		{/each}
	{/if}
</StreamPanel>
