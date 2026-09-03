<script lang="ts">
	import CheckIcon from '@lucide/svelte/icons/check';
	import { MODES } from '$lib/protocol/modes';
	import { MODE_NAMES } from '$lib/protocol/blob';
	import { device } from '$lib/state/device.svelte';
	import { FIELD, OP } from '$lib/protocol/fields';
	import Panel from '$lib/components/Panel.svelte';
	import Toggle from '$lib/components/Toggle.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';

	const status = $derived(device.status);

	const TAG_CLASS: Record<string, string> = {
		default: 'bg-primary-500/15 text-primary-700-300',
		console: 'bg-success-500/15 text-success-700-300',
		'PC only': 'bg-warning-500/15 text-warning-700-300',
	};
	const tagClass = (t: string) => TAG_CLASS[t] ?? 'bg-app-hover text-app-muted';

	// The face chords skip modes 7 and 8; the D-pad chords offer every mode.
	// Those two drop WebUSB entirely, so a chord is the only way into them --
	// and back4+A is always the way back out.
	const FACE_OPTIONS = MODE_NAMES.map((name, id) => ({ id, name })).filter((m) => m.id !== 7 && m.id !== 8);
	const DPAD_OPTIONS = MODE_NAMES.map((name, id) => ({ id, name }));
</script>

<div class="space-y-4">
	<Panel title="USB mode">
		{#snippet info()}
			<InfoPopover title="USB mode">
				Switching reboots the copycat and re-enumerates USB (~2 s). WebUSB works in every mode, including Steam/Lizard
				with Steam running (Chrome claims the vendor interface; Steam keeps the HID slots).
			</InfoPopover>
		{/snippet}

		<!-- 11 naked buttons became cards: the selected one is obvious, and each
		     mode carries its own description instead of two shared paragraphs. -->
		<div class="grid gap-2.5 autofill-[230px]">
			{#each MODES as m (m.id)}
				{@const active = status?.mode === m.id}
				<button
					type="button"
					onclick={() => device.sendRaw([OP.setMode, m.id])}
					class="rounded-container flex flex-col border p-3 text-left transition-colors
						{active
						? 'border-success-500 bg-success-500/10 ring-success-500/30 ring-1'
						: 'border-app-line bg-app-well hover:border-primary-600'}"
				>
					<div class="flex items-start gap-1.5">
						<span class="text-sm font-semibold">{m.name}</span>
						{#if active}
							<span
								class="bg-success-500 text-success-contrast-500 ml-auto flex shrink-0 items-center gap-0.5 rounded-full py-0.5 pr-1.5 pl-1 text-[10px] font-bold"
							>
								<CheckIcon size={11} /> active
							</span>
						{/if}
					</div>
					<p class="text-app-muted mt-1 text-xs leading-snug">{m.summary}</p>
					<div class="mt-auto flex flex-wrap items-center gap-1 pt-2">
						{#each m.tags as t (t)}
							<span class="rounded px-1.5 py-0.5 text-[10px] font-medium {tagClass(t)}">{t}</span>
						{/each}
						{#if m.detail}
							<span class="ml-auto"><InfoPopover title={m.name}>{m.detail}</InfoPopover></span>
						{/if}
					</div>
				</button>
			{/each}
		</div>

		<div class="border-app-line mt-4 flex items-center gap-3 border-t pt-3">
			<Toggle
				label="Persist last mode"
				checked={!!status?.persistMode}
				disabled={!status}
				onChange={(next) => device.setField(FIELD.persistMode, next ? 1 : 0)}
			/>
			<InfoPopover title="Persist last mode">
				Off (default): every restart / fresh reconnect boots into Steam mode. On: remembers the last mode you selected.
			</InfoPopover>
		</div>
	</Panel>

	<Panel title="Back4 chords">
		{#snippet info()}
			<InfoPopover title="Back4 chords">
				Hold all four back paddles (L4+R4+L5+R5) plus a face button or D-pad direction to switch mode without the WebUI.
			</InfoPopover>
		{/snippet}

		<table class="w-full text-sm">
			<thead>
				<tr class="text-app-muted border-app-line border-b text-left text-[11px] uppercase">
					<th class="w-40 pb-1.5 font-medium">Chord</th>
					<th class="pb-1.5 font-medium">Mode</th>
				</tr>
			</thead>
			<tbody>
				<!-- back4+A is fixed in firmware: it is the way out of every other
				     mode, so it is shown rather than left implicit. -->
				<tr class="border-app-line-soft border-b">
					<td class="py-2 font-mono text-xs">back4 + A</td>
					<td class="text-app-muted py-2">Steam (puck) <span class="text-[11px]">— always, not configurable</span></td>
				</tr>
				{#each ['B', 'X', 'Y'] as b, i (b)}
					<tr class="border-app-line-soft border-b">
						<td class="py-2 font-mono text-xs">back4 + {b}</td>
						<td class="py-2">
							<select
								class="select w-full max-w-xs text-sm"
								value={status?.chords.face[i]}
								disabled={!status}
								onchange={(e) => device.setField(FIELD.chordFace[i], +e.currentTarget.value)}
							>
								{#each FACE_OPTIONS as o (o.id)}
									<option value={o.id}>{o.name}</option>
								{/each}
							</select>
						</td>
					</tr>
				{/each}
				{#each ['Left', 'Up', 'Right', 'Down'] as d, i (d)}
					<tr class="border-app-line-soft border-b" class:opacity-50={!status?.caps.dpadChords}>
						<td class="py-2 font-mono text-xs">
							back4 + {d}
							{#if status && !status.caps.dpadChords}
								<span class="text-warning-700-300 ml-1 text-[10px]">fw ≥ v18</span>
							{/if}
						</td>
						<td class="py-2">
							<select
								class="select w-full max-w-xs text-sm"
								value={status?.chords.dpad[i]}
								disabled={!status?.caps.dpadChords}
								onchange={(e) => device.setField(FIELD.chordDpad[i], +e.currentTarget.value)}
							>
								{#each DPAD_OPTIONS as o (o.id)}
									<option value={o.id}>{o.name}</option>
								{/each}
							</select>
						</td>
					</tr>
				{/each}
			</tbody>
		</table>
	</Panel>
</div>
