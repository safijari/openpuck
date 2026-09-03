<script lang="ts">
	import ChevronsLeftIcon from '@lucide/svelte/icons/chevrons-left';
	import ChevronsRightIcon from '@lucide/svelte/icons/chevrons-right';
	import { DONGLE_SECTIONS, SECTIONS, ui, type SectionId } from '$lib/state/ui.svelte';
	import { device } from '$lib/state/device.svelte';
	import ConfirmDialog, { type ConfirmSpec } from '$lib/components/ConfirmDialog.svelte';

	let confirming = $state<ConfirmSpec | null>(null);

	// A beta-gated section asks first and enables beta on accept, rather than
	// just wearing a label. Flashing can leave a puck needing UF2 DFU to
	// recover, so the warning has to be acknowledged, not merely displayed.
	function go(id: SectionId, locked: boolean) {
		if (!locked) {
			ui.go(id);
			return;
		}
		confirming = {
			title: 'Firmware update is beta',
			body: [
				'Firmware update is still being tested and may not work correctly.',
				'A bad image or interrupted recovery path can leave you needing UF2 DFU to recover.',
				'Enable beta mode before using this feature.',
			],
			confirmLabel: 'Enable beta',
			onConfirm: () => {
				ui.beta = true;
				ui.go(id);
			},
		};
	}
</script>

<nav
	class="border-app-line bg-app-chrome hidden shrink-0 flex-col gap-1 border-r p-2 transition-[width] lg:flex
		{ui.railCollapsed ? 'w-14' : 'w-14 xl:w-44'}"
	aria-label="Sections"
>
	{#each SECTIONS.filter((s) => !device.isDongle || DONGLE_SECTIONS.includes(s.id)) as s (s.id)}
		{@const dimmed = !device.connected && s.id !== 'sniffer'}
		<!-- Flashing a dongle is a primary use, so its Firmware section is not
		     beta-gated the way a puck's is. -->
		{@const locked = s.beta && !ui.beta && !device.isDongle}
		{@const SectionIcon = s.icon}
		<button
			type="button"
			onclick={() => go(s.id, !!locked)}
			aria-current={ui.section === s.id ? 'page' : undefined}
			title={ui.railCollapsed ? s.label : undefined}
			class="rounded-base flex items-center gap-2.5 px-2.5 py-2 text-left text-sm transition-colors
				{dimmed ? 'opacity-50' : ''}
				{ui.section === s.id
				? 'bg-primary-500/15 text-primary-700-300 ring-primary-500/40 ring-1'
				: 'text-app-muted hover:bg-app-card hover:text-app-strong'}"
		>
			<SectionIcon size={16} class="shrink-0" />
			{#if !ui.railCollapsed}
				<span class="hidden truncate xl:inline">{s.label}</span>
				{#if locked}
					<span class="text-warning-700-300 ml-auto hidden text-[10px] font-semibold tracking-wide uppercase xl:inline">
						beta
					</span>
				{/if}
			{/if}
		</button>
	{/each}

	<button
		type="button"
		onclick={() => (ui.railCollapsed = !ui.railCollapsed)}
		class="text-app-muted hover:text-app-strong rounded-base mt-auto hidden items-center gap-2.5 px-2.5 py-2 text-left text-xs xl:flex"
		aria-label={ui.railCollapsed ? 'Expand sidebar' : 'Collapse sidebar'}
	>
		{#if ui.railCollapsed}
			<ChevronsRightIcon size={16} />
		{:else}
			<ChevronsLeftIcon size={16} />
			<span>Collapse</span>
		{/if}
	</button>
</nav>

<ConfirmDialog spec={confirming} onCancel={() => (confirming = null)} />
