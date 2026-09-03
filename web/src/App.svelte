<script lang="ts">
	import AppBar from '$lib/components/AppBar.svelte';
	import Rail from '$lib/components/Rail.svelte';
	import MonitorSidebar from '$lib/components/MonitorSidebar.svelte';
	import LogPanel from '$lib/panels/LogPanel.svelte';
	import CapturePanel from '$lib/panels/CapturePanel.svelte';
	import TrailPanel from '$lib/panels/TrailPanel.svelte';
	import UpdateModal from '$lib/components/UpdateModal.svelte';
	import Overview from '$lib/sections/Overview.svelte';
	import Modes from '$lib/sections/Modes.svelte';
	import Mapping from '$lib/sections/Mapping.svelte';
	import Feel from '$lib/sections/Feel.svelte';
	import Desktop from '$lib/sections/Desktop.svelte';
	import { ui, type SectionId } from '$lib/state/ui.svelte';
	import type { Component } from 'svelte';
	import { device, supported } from '$lib/state/device.svelte';
	import { logs } from '$lib/state/log.svelte';
	import TriangleAlertIcon from '@lucide/svelte/icons/triangle-alert';

	const EAGER = {
		overview: Overview,
		modes: Modes,
		mapping: Mapping,
		feel: Feel,
		desktop: Desktop,
	};

	/**
	 * Firmware (UF2 parsing, CRC, the release list), Diagnostics and the sniffer
	 * are occasional and carry most of the weight, so they are fetched the first
	 * time they are opened rather than on every page load.
	 */
	const LAZY: Record<string, () => Promise<{ default: Component }>> = {
		firmware: () => import('$lib/sections/Firmware.svelte'),
		diagnostics: () => import('$lib/sections/Diagnostics.svelte'),
		sniffer: () => import('$lib/sections/Sniffer.svelte'),
	};

	const Current = $derived(EAGER[ui.section as keyof typeof EAGER]);

	/** Sections with nothing to show until a puck is attached. */
	const NEEDS_PUCK = new Set<SectionId>(['overview', 'modes', 'mapping', 'feel', 'desktop', 'firmware', 'diagnostics']);

	// One-shot bootstrap. In an $effect this would read and write the same state
	// and loop forever.
	const fixture = new URLSearchParams(location.search).get('fixture');
	if (fixture === 'dongle') {
		device.loadDongleFixture();
		logs.ok('loaded ReversePuck fixture (no device attached)');
	} else if (fixture === 'true') {
		device.loadFixture();
		logs.ok('loaded v17 fixture (no device attached)');
	} else if (supported) {
		device.init();
	}
	ui.listenToHash();
</script>

<div class="bg-app-bg text-app-strong flex h-screen flex-col overflow-hidden">
	<AppBar />

	{#if !supported}
		<div class="flex flex-1 items-center justify-center p-8">
			<div class="max-w-md text-center">
				<h1 class="mb-2 text-lg font-semibold">This browser can't talk to the puck</h1>
				<p class="text-app-muted text-sm">
					The panel needs WebUSB, which is available in Chrome and Edge. Firefox and Safari do not implement it.
				</p>
			</div>
		</div>
	{:else}
		<div class="flex min-h-0 flex-1 flex-col lg:flex-row">
			<!-- The rail stays available with no puck attached: the sniffer is a
			     separate board and must be reachable on its own. -->
			<Rail />
			<main class="min-w-0 flex-1 overflow-y-auto p-3 sm:p-4">
				<div class="mx-auto max-w-[1600px]">
					{#if device.connected || !NEEDS_PUCK.has(ui.section)}
						<!--
							Keyed on the section so moving away from a broken one clears
							the error. Without a boundary a single render failure blanks
							the whole panel silently, which is the worst way for a tool
							you reach for when hardware is already misbehaving to fail.
						-->
						{#key ui.section}
							<svelte:boundary onerror={(e) => logs.error(`section "${ui.section}" failed: ${(e as Error).message}`)}>
								{#if Current}
									<Current />
								{:else}
									{#await LAZY[ui.section]()}
										<p class="text-app-muted p-6 text-sm">Loading…</p>
									{:then mod}
										<mod.default />
									{:catch err}
										<p class="text-error-700-300 p-6 text-sm">Could not load this section: {err.message}</p>
									{/await}
								{/if}

								{#snippet failed(error, reset)}
									<div class="border-error-500 bg-error-500/10 rounded-container border p-4">
										<h2 class="text-error-700-300 flex items-center gap-2 text-sm font-semibold">
											<TriangleAlertIcon size={16} /> This section failed to render
										</h2>
										<p class="text-app-muted mt-1 text-xs">
											The rest of the panel still works, and the device connection is unaffected. The error is in the
											log.
										</p>
										<pre
											class="bg-app-well border-app-line rounded-base mt-2 overflow-auto border p-2 font-mono text-xs">{(
												error as Error
											).message}</pre>
										<button type="button" class="btn preset-tonal-surface btn-sm mt-3" onclick={reset}>
											Try again
										</button>
									</div>
								{/snippet}
							</svelte:boundary>
						{/key}
					{:else}
						<div class="flex flex-col items-center justify-center py-24 text-center">
							<h1 class="mb-2 text-lg font-semibold">Connect your puck</h1>
							<p class="text-app-muted mb-4 max-w-lg text-sm">
								First connect uses the Chrome/Edge device picker; reconnects are automatic. If the picker is empty, quit
								any app holding the device or replug.
							</p>
							<button type="button" class="btn preset-filled-primary-500" onclick={() => device.connect()}>
								{device.conn === 'connecting' ? 'Connecting…' : 'Connect'}
							</button>
							<p class="text-app-muted mt-4 max-w-lg text-xs">
								On Linux, a puck that stays "disconnected" after the picker is usually a udev permissions issue — see <a
									class="text-primary-700-300 hover:underline"
									href="./WEBUSB_LINUX.md">WEBUSB_LINUX.md</a
								>.
							</p>
							{#if ui.section !== 'sniffer'}
								<p class="text-app-muted mt-6 text-xs">
									Using the RF sniffer instead? It's a separate board —
									<button type="button" class="text-primary-700-300 underline" onclick={() => ui.go('sniffer')}>
										open the sniffer
									</button>.
								</p>
							{/if}
						</div>
					{/if}
				</div>
			</main>
			{#if ui.monitorOpen && device.connected && !device.isDongle && ui.section !== 'sniffer'}
				<MonitorSidebar />
			{/if}
		</div>
	{/if}

	<LogPanel />
	<CapturePanel />
	<TrailPanel />
	<UpdateModal />
</div>
