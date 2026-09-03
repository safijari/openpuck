<script lang="ts">
	import ArrowRightIcon from '@lucide/svelte/icons/arrow-right';
	import { device } from '$lib/state/device.svelte';
	import { ui } from '$lib/state/ui.svelte';
	import Stat from './Stat.svelte';

	const status = $derived(device.status);
	const slots = $derived(device.bondedSlots);
	const slot = $derived(status?.slots[device.activeSlot] ?? null);
	const stats = $derived(slot?.stats);
</script>

<!--
	The always-visible half of what used to be one 816px "Link status" card
	buried as the 4th of 12 cards. These six are the glanceable per-controller
	readings; the deep global telemetry lives in Diagnostics.
-->
<aside class="border-app-line bg-app-chrome shrink-0 overflow-y-auto border-t p-3 lg:w-72 lg:border-t-0 lg:border-l">
	<h2 class="text-app-muted mb-2 text-xs font-semibold tracking-[0.08em] uppercase">Link status</h2>

	{#if !status}
		<p class="text-app-muted text-sm">No device connected.</p>
	{:else}
		{#if slots.length > 1}
			<div class="mb-3 flex flex-wrap gap-1.5">
				{#each slots as s (s)}
					{@const up = status.slots[s]?.up}
					<button
						type="button"
						onclick={() => (device.activeSlot = s)}
						class="rounded-base flex items-center gap-1.5 border px-2.5 py-1 text-xs font-semibold transition-colors
							{device.activeSlot === s
							? 'border-primary-500 bg-primary-500/15 text-app-strong'
							: 'border-app-line bg-app-well text-app-muted hover:border-primary-600'}"
					>
						<span class="size-1.5 rounded-full {up ? 'bg-success-400' : 'bg-error-500'}"></span>
						{s + 1}
					</button>
				{/each}
			</div>
		{/if}

		<div class="grid grid-cols-3 gap-2 sm:grid-cols-4 lg:grid-cols-2">
			<Stat label="RF link" value={slot?.up ? 'up' : 'down'} tone={slot?.up ? 'up' : 'down'} />
			<Stat label="Battery" value={slot?.up && slot.battery ? `${slot.battery}%` : null} />
			<Stat label="Signal" value={slot?.up && slot.rssi ? `-${slot.rssi} dBm` : null} />
			<Stat label="Polls/s" value={stats ? `${stats.polls} /s` : null} />
			<Stat label="Delivered" value={stats ? `${stats.f1} /s` : null} />
			<Stat
				label="Fails/s"
				value={stats ? `${stats.crc} · ${stats.norx} · ${stats.relay}` : null}
				hint="crc · noRx · relay"
			/>
		</div>

		<div class="border-app-line-soft mt-3 space-y-1.5 border-t pt-3 text-xs">
			<div class="flex items-center justify-between gap-2">
				<span class="text-app-muted">Loop</span>
				{#if status.loop.stalled}
					<span class="bg-error-100-900 text-error-700-300 rounded-full px-2 py-0.5 font-semibold">
						STALLED @ {status.loop.stage}
						{status.loop.stallMs}ms
					</span>
				{:else}
					<span class="bg-success-100-900 text-success-700-300 rounded-full px-2 py-0.5 font-semibold">running</span>
				{/if}
			</div>
			<div class="flex items-center justify-between gap-2">
				<span class="text-app-muted">Last reset</span>
				<span class:text-error-700-300={status.reset?.isFault} class="truncate">{status.reset?.name ?? '—'}</span>
			</div>
			<button
				type="button"
				onclick={() => ui.go('diagnostics')}
				class="text-primary-700-300 hover:text-primary-600-400 flex items-center gap-1 pt-1 text-xs"
			>
				All telemetry <ArrowRightIcon size={12} />
			</button>
		</div>
	{/if}
</aside>
