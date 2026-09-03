<script lang="ts">
	import { device } from '$lib/state/device.svelte';
	import { FIELD, GYRO_MAPS, OP, RUMBLE_SCALES, RUMBLE_STYLES } from '$lib/protocol/fields';
	import { logs } from '$lib/state/log.svelte';
	import Panel from '$lib/components/Panel.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import GateBadge from '$lib/components/GateBadge.svelte';

	const status = $derived(device.status);

	// The trackpad cursor only applies in Xbox (1) and Lizard (3); the original
	// dimmed the whole card in every other mode rather than hiding it.
	const mouseActive = $derived(status?.mode === 1 || status?.mode === 3);

	// A strength set from the serial console can land between the presets, so
	// show the nearest one rather than an empty select.
	const nearestScale = $derived.by(() => {
		const want = status?.rumbleShaping.scalePct ?? 200;
		return RUMBLE_SCALES.reduce((a, b) => (Math.abs(b - want) < Math.abs(a - want) ? b : a));
	});

	async function testRumble() {
		await device.sendRaw([OP.rumbleTest]);
		logs.info(`test rumble sent (style ${status?.rumbleShaping.style ?? 0}, ${nearestScale}%)`);
	}
</script>

<div class="grid gap-4 autofit-[380px]">
	<Panel title="Trackpad mouse" class={mouseActive ? '' : 'opacity-60'}>
		{#snippet info()}
			<InfoPopover title="Trackpad mouse (lizard + Xbox)">
				Right-pad cursor in Steam-mode lizard (Steam closed) and in Xbox mode. Sensitivity is a divisor — lower = faster
				pointer. Glide = how long the cursor coasts after a flick.
			</InfoPopover>
		{/snippet}

		{#if !mouseActive}
			<p class="text-app-muted mb-3 text-xs">
				Inactive in {status?.modeName ?? 'this mode'} — applies to Xbox and Lizard.
			</p>
		{/if}

		<label class="mb-3 block">
			<div class="mb-1 flex items-center justify-between text-sm">
				<span>Sensitivity</span>
				<span class="text-secondary-700-300 tabular-nums">{status?.mouse.div ?? '—'}</span>
			</div>
			<input
				type="range"
				min="4"
				max="200"
				step="1"
				class="w-full"
				value={status?.mouse.div ?? 32}
				disabled={!status}
				onchange={(e) => device.setField(FIELD.mouseDiv, +e.currentTarget.value)}
			/>
		</label>

		<label class="block">
			<div class="mb-1 flex items-center justify-between text-sm">
				<span>Glide / friction</span>
				<span class="text-secondary-700-300 tabular-nums">{status?.mouse.friction ?? '—'}</span>
			</div>
			<input
				type="range"
				min="0"
				max="99"
				step="1"
				class="w-full"
				value={status?.mouse.friction ?? 40}
				disabled={!status}
				onchange={(e) => device.setField(FIELD.mouseFriction, +e.currentTarget.value)}
			/>
		</label>
	</Panel>

	<Panel title="Switch Pro motion">
		{#snippet info()}
			<InfoPopover title="Gyro mapping">
				Corrected (default) trims the gyro so Switch Pro mode matches a genuine Pro Controller and Steam mode (roll
				×0.8, pitch/yaw ×0.9). Legacy sends the raw axes untrimmed — the pre-fix behavior, if you preferred the faster
				feel or tuned your in-game sensitivity around it.
			</InfoPopover>
		{/snippet}

		<div class="flex items-center gap-2">
			<span class="w-32 shrink-0 text-sm">Gyro mapping</span>
			<select
				class="select flex-1 text-sm"
				value={status?.gyroLegacy ? 1 : 0}
				disabled={!status?.caps.gyroMap}
				onchange={(e) => device.setField(FIELD.swGyroMap, +e.currentTarget.value)}
			>
				{#each GYRO_MAPS as [label, v] (v)}
					<option value={v}>{label}</option>
				{/each}
			</select>
			<GateBadge ok={status?.caps.gyroMap} requires="v19" />
		</div>
	</Panel>

	<Panel title="Rumble">
		{#snippet info()}
			<InfoPopover title="Rumble shaping">
				Applies to the translated modes (Xbox / Switch / PlayStation), where the puck decodes the host's rumble packet
				itself. Steam mode relays Steam's own haptics untouched, so these do nothing there. Strength is a percentage of
				the amplitude the game asked for; 200% is the shipped default, so leave it there for stock feel. Style reshapes
				the two motors before that scale: <strong>mono</strong> drives both at the stronger value,
				<strong>heavy</strong>
				and <strong>light</strong> mute one motor each,
				<strong>punchy</strong> softens weak effects while leaving strong ones alone, and <strong>soft</strong> lifts weak
				ones so subtle rumble is felt.
			</InfoPopover>
		{/snippet}

		<div class="mb-2 flex items-center gap-2">
			<span class="w-32 shrink-0 text-sm">Style</span>
			<select
				class="select flex-1 text-sm"
				value={status?.rumbleShaping.style ?? 0}
				disabled={!status?.caps.rumble}
				onchange={(e) => device.setField(FIELD.rumbleStyle, +e.currentTarget.value)}
			>
				{#each RUMBLE_STYLES as [label, v] (v)}
					<option value={v}>{label}</option>
				{/each}
			</select>
			<GateBadge ok={status?.caps.rumble} requires="v21" />
		</div>

		<div class="mb-3 flex items-center gap-2">
			<span class="w-32 shrink-0 text-sm">Strength</span>
			<select
				class="select flex-1 text-sm"
				value={nearestScale}
				disabled={!status?.caps.rumble}
				onchange={(e) => device.setField(FIELD.rumbleScale, +e.currentTarget.value / 2)}
			>
				{#each RUMBLE_SCALES as v (v)}
					<option value={v}>{v}%</option>
				{/each}
			</select>
		</div>

		<div class="flex items-center gap-3">
			<button type="button" class="btn preset-tonal-surface btn-sm" disabled={!device.connected} onclick={testRumble}>
				Test rumble
			</button>
			<span class="text-app-muted text-xs">
				Buzzes every connected controller for half a second at the settings above, so you can compare styles without
				launching a game.
			</span>
		</div>
	</Panel>
</div>
