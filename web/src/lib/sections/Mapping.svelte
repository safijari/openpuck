<script lang="ts">
	import { device } from '$lib/state/device.svelte';
	import { padStickField, typeField, TYPE_OFF } from '$lib/protocol/fields';
	import {
		BACK_LABELS,
		etypeForMode,
		PAD_STICK_LABELS,
		PAD_STICK_OPTS,
		targetOptions,
		TYPE_DEFS,
	} from '$lib/protocol/types';
	import { Tabs } from '@skeletonlabs/skeleton-svelte';
	import Panel from '$lib/components/Panel.svelte';
	import Toggle from '$lib/components/Toggle.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import GateBadge from '$lib/components/GateBadge.svelte';

	const status = $derived(device.status);
	const activeEt = $derived(status ? etypeForMode(status.mode) : -1);

	// Open the tab matching the current mode on first load, then leave the
	// user's choice alone -- switching modes mid-edit must not yank the tab.
	let tab = $state<number | null>(null);
	$effect(() => {
		if (tab === null && activeEt >= 0) tab = activeEt;
	});
	const current = $derived(tab ?? 0);

	const cfg = $derived(status?.types[current]);
	const def = $derived(TYPE_DEFS[current]);
	const backOptions = $derived(targetOptions(def, true));
	const qamOptions = $derived(targetOptions(def, false));

	const set = (off: number, v: number) => device.setField(typeField(current, off), v);
	const ledLabel = (v: number) => (v === 0 ? 'Auto' : `${v}%`);
</script>

<Panel title="Button mapping">
	{#snippet info()}
		<InfoPopover title="Button mapping">
			Separate mappings per emulated controller type — each only offers targets that exist on that controller. The type
			matching the current mode is marked. Steam &amp; Lizard are native puck modes (Steam does its own remapping) and
			aren't configured here.
		</InfoPopover>
	{/snippet}

	<Tabs value={String(current)} onValueChange={(e) => (tab = Number(e.value))} class="mb-4">
		<Tabs.List class="flex flex-wrap gap-1.5">
			{#each TYPE_DEFS as t, et (t.key)}
				<Tabs.Trigger
					value={String(et)}
					class="rounded-base flex items-center gap-1.5 border px-3 py-1.5 text-sm font-semibold transition-colors
						{current === et
						? 'border-primary-500 bg-primary-500/15 text-app-strong'
						: 'border-app-line bg-app-well text-app-muted hover:border-primary-600'}"
				>
					{t.name}
					{#if et === activeEt}
						<span class="bg-success-500 size-1.5 shrink-0 rounded-full" title="Matches the current USB mode"></span>
					{/if}
				</Tabs.Trigger>
			{/each}
		</Tabs.List>
	</Tabs>

	{#if !cfg}
		<p class="text-app-muted text-sm">
			{status ? 'This firmware does not report per-type config.' : 'Connect a puck to configure mappings.'}
		</p>
	{:else}
		<!-- Full width lets these lay out side by side instead of stacking as
		     rows in a 980px column. -->
		<div class="grid gap-x-8 gap-y-3 autofit-[320px]">
			<div class="space-y-3">
				<div class="text-app-muted text-[11px] font-semibold tracking-wider uppercase">Back paddles</div>
				{#each BACK_LABELS as label, i (label)}
					<label class="flex items-center gap-3">
						<span class="w-44 shrink-0 text-sm">{label}</span>
						<select
							class="select flex-1 text-sm"
							value={cfg.back[i]}
							onchange={(e) => set(TYPE_OFF.back + i, +e.currentTarget.value)}
						>
							{#each backOptions as [v, l] (v)}
								<option value={v}>{l}</option>
							{/each}
						</select>
					</label>
				{/each}

				<label class="flex items-center gap-3">
					<span class="w-44 shrink-0 text-sm">QAM (3 dots)</span>
					<select
						class="select flex-1 text-sm"
						value={cfg.qam}
						onchange={(e) => set(TYPE_OFF.qam, +e.currentTarget.value)}
					>
						{#each qamOptions as [v, l] (v)}
							<option value={v}>{l}</option>
						{/each}
					</select>
				</label>
			</div>

			<div class="space-y-3">
				<div class="text-app-muted text-[11px] font-semibold tracking-wider uppercase">Behaviour</div>

				{#each [['A/B + X/Y swap', TYPE_OFF.abSwap, cfg.abSwap], ['Trackpad haptics', TYPE_OFF.padHaptics, cfg.padHaptics], ['Rumble', TYPE_OFF.rumble, cfg.rumble]] as [label, off, on] (label)}
					<Toggle label={label as string} checked={!!on} onChange={(next) => set(off as number, next ? 1 : 0)} />
				{/each}

				<label class="flex items-center gap-3">
					<span class="w-44 shrink-0 text-sm">LED brightness</span>
					<input
						type="range"
						min="0"
						max="100"
						step="5"
						class="flex-1"
						value={cfg.led}
						onchange={(e) => set(TYPE_OFF.led, +e.currentTarget.value)}
					/>
					<span class="text-secondary-700-300 tabular-nums w-12 shrink-0 text-right text-sm">{ledLabel(cfg.led)}</span>
				</label>

				<div class="flex items-center gap-2 pt-1">
					<div class="text-app-muted text-[11px] font-semibold tracking-wider uppercase">Trackpad → stick</div>
					<GateBadge ok={status?.caps.padStick} requires="v20" />
				</div>
				{#each PAD_STICK_LABELS as label, pad (label)}
					<label class="flex items-center gap-3">
						<span class="w-44 shrink-0 text-sm">{label}</span>
						<select
							class="select flex-1 text-sm"
							value={cfg.padStick[pad]}
							disabled={!status?.caps.padStick}
							onchange={(e) => device.setField(padStickField(current, pad as 0 | 1), +e.currentTarget.value)}
						>
							{#each PAD_STICK_OPTS as [v, l] (v)}
								<option value={v}>{l}</option>
							{/each}
						</select>
					</label>
				{/each}
			</div>
		</div>
	{/if}
</Panel>
