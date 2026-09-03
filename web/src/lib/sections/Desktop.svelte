<script lang="ts">
	import CircleAlertIcon from '@lucide/svelte/icons/circle-alert';
	import KeyboardIcon from '@lucide/svelte/icons/keyboard';
	import MousePointerClickIcon from '@lucide/svelte/icons/mouse-pointer-click';
	import MoveIcon from '@lucide/svelte/icons/move';
	import PencilIcon from '@lucide/svelte/icons/pencil';
	import PlusIcon from '@lucide/svelte/icons/plus';
	import RotateCcwIcon from '@lucide/svelte/icons/rotate-ccw';
	import SaveIcon from '@lucide/svelte/icons/save';
	import ScrollTextIcon from '@lucide/svelte/icons/scroll-text';
	import Trash2Icon from '@lucide/svelte/icons/trash-2';
	import Volume2Icon from '@lucide/svelte/icons/volume-2';
	import XIcon from '@lucide/svelte/icons/x';
	import { device } from '$lib/state/device.svelte';
	import {
		bindingProblem,
		defaultPayload,
		describeOutput,
		describeTrigger,
		isAnalog,
		LZ_AXIS_SRC,
		LZ_BTNS,
		LZ_CONSUMER,
		LZ_GYRO_ACT,
		LZ_KEYS,
		LZ_MAX,
		LZ_MBTNS,
		LZ_MODS,
		LZ_OUT_LABELS,
		LZO,
	} from '$lib/protocol/lizard';
	import Panel from '$lib/components/Panel.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import GateBadge from '$lib/components/GateBadge.svelte';

	const status = $derived(device.status);
	const bindings = $derived(device.lizard);
	const capable = $derived(device.lizardCapable);

	const TRIG_OPTS: [number, string][] = [[0, '— pick input —'], ...LZ_BTNS];
	const HOLD_OPTS: [number, string][] = [[0, '(none)'], ...LZ_BTNS];

	const KIND_ICON = {
		[LZO.KBD]: KeyboardIcon,
		[LZO.MBTN]: MousePointerClickIcon,
		[LZO.AXIS]: MoveIcon,
		[LZO.SCROLL]: ScrollTextIcon,
		[LZO.CONSUMER]: Volume2Icon,
	} as Record<number, typeof KeyboardIcon>;

	/** Only one row is editable at a time; the rest stay readable. */
	let editing = $state<number | null>(null);
	let confirmReset = $state(false);

	function addBinding() {
		if (bindings.length >= LZ_MAX) return;
		device.lizard.push({ outType: LZO.KBD, od: defaultPayload(LZO.KBD), trig: 0, hold: 0 });
		editing = device.lizard.length - 1;
	}

	function remove(i: number) {
		device.lizard.splice(i, 1);
		if (editing === i) editing = null;
		else if (editing !== null && editing > i) editing--;
	}

	function setOutType(i: number, v: number) {
		device.lizard[i].outType = v;
		device.lizard[i].od = defaultPayload(v);
	}

	function setMod(i: number, bit: number, on: boolean) {
		const od = device.lizard[i].od;
		od[0] = on ? od[0] | bit : od[0] & ~bit;
	}
</script>

{#snippet field(label: string)}
	<div class="text-app-faint mb-1 text-[10px] font-semibold tracking-wider uppercase">{label}</div>
{/snippet}

<Panel title="Lizard / desktop mapping">
	{#snippet info()}
		<InfoPopover title="Lizard / desktop mapping">
			Keyboard, mouse &amp; media bindings for <strong>Lizard (always)</strong> mode — each row maps a controller input
			to a desktop action. (Steam-mode seamless lizard, when Steam is closed, keeps the built-in default behavior and is
			not affected by this map.) Edits apply after <strong>Save to device</strong>.
		</InfoPopover>
	{/snippet}

	{#snippet actions()}
		<GateBadge ok={capable} requires="v16" />
	{/snippet}

	{#if !status}
		<p class="text-app-muted text-sm">Connect a puck to edit desktop bindings.</p>
	{:else if !capable}
		<p class="text-app-muted text-sm">
			This puck's firmware predates the desktop map (needs status protocol v16). Update it to configure bindings.
		</p>
	{:else}
		<div class="mb-3 flex flex-wrap items-center gap-2">
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
				disabled={bindings.length >= LZ_MAX}
				onclick={addBinding}
			>
				<PlusIcon size={14} /> Add binding
			</button>
			<button
				type="button"
				class="btn preset-filled-primary-500 btn-sm flex items-center gap-1.5"
				disabled={device.busy.lizard}
				onclick={() => device.saveLizard()}
			>
				<SaveIcon size={14} /> Save to device
			</button>
			<button type="button" class="btn preset-tonal-surface btn-sm" onclick={() => device.loadLizard()}>Reload</button>
			<span class="text-app-muted ml-auto text-xs tabular-nums">{bindings.length} / {LZ_MAX}</span>
			{#if confirmReset}
				<span class="text-app-muted text-xs">Reset to defaults? Unsaved edits are lost.</span>
				<button
					type="button"
					class="btn preset-filled-error-500 btn-sm"
					onclick={() => {
						confirmReset = false;
						editing = null;
						device.resetLizard();
					}}
				>
					Reset
				</button>
				<button type="button" class="btn preset-tonal-surface btn-sm" onclick={() => (confirmReset = false)}>
					Cancel
				</button>
			{:else}
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
					onclick={() => (confirmReset = true)}
				>
					<RotateCcwIcon size={14} /> Reset to defaults
				</button>
			{/if}
		</div>

		{#if bindings.length === 0}
			<p class="text-app-muted py-6 text-center text-sm">
				No bindings yet. <button type="button" class="text-primary-700-300 underline" onclick={addBinding}>
					Add one
				</button> to map a controller input to a desktop action.
			</p>
		{/if}

		<div class="border-app-line divide-app-line-soft divide-y rounded-base border">
			{#each bindings as b, i (i)}
				{@const open = editing === i}
				{@const problem = bindingProblem(b)}
				{@const Icon = KIND_ICON[b.outType]}
				<div class:bg-app-well={open}>
					<!-- Collapsed: the binding as a sentence, aligned into columns so
					     32 of them scan in one pass. -->
					<div class="flex items-center gap-3 px-3 py-2">
						<span class="text-app-faint shrink-0">
							{#if Icon}<Icon size={15} />{:else}<CircleAlertIcon size={15} />{/if}
						</span>

						<span class="w-56 shrink-0 truncate text-sm" class:text-app-faint={!describeTrigger(b)}>
							{describeTrigger(b) || 'no trigger'}
						</span>

						<span class="text-app-faint shrink-0 text-xs">→</span>

						<span class="min-w-0 flex-1 truncate text-sm font-medium">{describeOutput(b)}</span>

						{#if problem}
							<span
								class="bg-warning-100-900 text-warning-700-300 flex shrink-0 items-center gap-1 rounded px-1.5 py-0.5 text-[10px] font-semibold"
								title="Rows like this are skipped when saving."
							>
								<CircleAlertIcon size={11} />
								{problem}
							</span>
						{/if}

						<button
							type="button"
							class="text-app-muted hover:text-app-strong shrink-0 px-1"
							title={open ? 'Done' : 'Edit binding'}
							aria-label={open ? 'Done editing' : 'Edit binding'}
							onclick={() => (editing = open ? null : i)}
						>
							{#if open}<XIcon size={15} />{:else}<PencilIcon size={14} />{/if}
						</button>
						<button
							type="button"
							class="text-app-muted hover:text-error-700-300 shrink-0 px-1"
							title="Delete binding"
							aria-label="Delete binding"
							onclick={() => remove(i)}
						>
							<Trash2Icon size={15} />
						</button>
					</div>

					<!-- Expanded: a labelled grid with fixed columns, so the controls
					     can't reflow into a different shape per output type. -->
					{#if open}
						<div class="border-app-line-soft space-y-3 border-t px-3 py-3">
							<div class="grid gap-3 autofit-[200px]">
								{#if !isAnalog(b)}
									<div>
										{@render field('When')}
										<select
											class="select w-full text-sm"
											value={b.trig}
											onchange={(e) => (b.trig = +e.currentTarget.value >>> 0)}
										>
											{#each TRIG_OPTS as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
									<div>
										{@render field('While holding')}
										<select
											class="select w-full text-sm"
											value={b.hold}
											onchange={(e) => (b.hold = +e.currentTarget.value >>> 0)}
										>
											{#each HOLD_OPTS as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
								{/if}
								<div>
									{@render field('Output')}
									<select
										class="select w-full text-sm"
										value={b.outType}
										onchange={(e) => setOutType(i, +e.currentTarget.value)}
									>
										{#each LZ_OUT_LABELS as [v, l] (v)}
											<option value={v}>{l}</option>
										{/each}
									</select>
								</div>

								{#if b.outType === LZO.KBD}
									<div>
										{@render field('Key')}
										<select
											class="select w-full text-sm"
											value={b.od[1] ?? 0}
											onchange={(e) => {
												b.od[1] = +e.currentTarget.value;
												for (let k = 2; k < 7; k++) b.od[k] = 0;
											}}
										>
											{#each LZ_KEYS as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
								{:else if b.outType === LZO.MBTN}
									<div>
										{@render field('Button')}
										<select
											class="select w-full text-sm"
											value={b.od[0] || 1}
											onchange={(e) => (b.od[0] = +e.currentTarget.value)}
										>
											{#each LZ_MBTNS as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
								{:else if b.outType === LZO.AXIS}
									<div>
										{@render field('Source')}
										<select
											class="select w-full text-sm"
											value={b.od[0] || 0}
											onchange={(e) => (b.od[0] = +e.currentTarget.value)}
										>
											{#each LZ_AXIS_SRC as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
									{#if (b.od[0] || 0) === 2}
										<div>
											{@render field('Active')}
											<select
												class="select w-full text-sm"
												value={b.od[1] || 0}
												onchange={(e) => (b.od[1] = +e.currentTarget.value)}
											>
												{#each LZ_GYRO_ACT as [v, l] (v)}
													<option value={v}>{l}</option>
												{/each}
											</select>
										</div>
									{/if}
								{:else if b.outType === LZO.CONSUMER}
									<div>
										{@render field('Media key')}
										<select
											class="select w-full text-sm"
											value={b.od[0] || 1}
											onchange={(e) => (b.od[0] = +e.currentTarget.value)}
										>
											{#each LZ_CONSUMER as [v, l] (v)}
												<option value={v}>{l}</option>
											{/each}
										</select>
									</div>
								{/if}
							</div>

							{#if b.outType === LZO.KBD}
								<div>
									{@render field('Modifiers')}
									<div class="flex flex-wrap gap-3">
										{#each LZ_MODS as [bit, name] (bit)}
											<label class="flex items-center gap-1.5 text-sm">
												<input
													type="checkbox"
													class="checkbox"
													checked={!!(b.od[0] & bit)}
													onchange={(e) => setMod(i, bit, e.currentTarget.checked)}
												/>
												{name}
											</label>
										{/each}
									</div>
								</div>
							{:else if b.outType === LZO.SCROLL}
								<p class="text-app-muted text-xs">Driven by the left trackpad; there is nothing else to configure.</p>
							{/if}
						</div>
					{/if}
				</div>
			{/each}
		</div>
	{/if}
</Panel>
