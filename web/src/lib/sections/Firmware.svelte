<script lang="ts">
	import RefreshCwIcon from '@lucide/svelte/icons/refresh-cw';
	import TriangleAlertIcon from '@lucide/svelte/icons/triangle-alert';
	import UploadIcon from '@lucide/svelte/icons/upload';
	import { FileUpload } from '@skeletonlabs/skeleton-svelte';
	import { device } from '$lib/state/device.svelte';
	import { fwup } from '$lib/state/fwup.svelte';
	import { releases, relAsset, REL_REPO, type Release } from '$lib/state/releases.svelte';
	import { compareVersions, imagePanelUpdatable, uf2ToImage } from '$lib/protocol/firmware';
	import { logs } from '$lib/state/log.svelte';
	import ConfirmDialog, { type ConfirmSpec } from '$lib/components/ConfirmDialog.svelte';
	import Panel from '$lib/components/Panel.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';

	const status = $derived(device.status);
	// Panel updates need status protocol v15+. A dongle is always allowed: it
	// flashes through the same path but reports its own protocol.
	const gated = $derived(!!status && !device.isDongle && !status.caps.panelUpdate);

	let selected = $state<{ name: string; image: Uint8Array } | null>(null);
	let factory = $state<Record<string, boolean>>({});
	let pending = $state<{ title: string; confirm: string; get: () => Promise<Uint8Array> } | null>(null);

	async function pickFile(f: File | undefined) {
		if (!f) return;
		try {
			selected = { name: f.name, image: uf2ToImage(await f.arrayBuffer()) };
			logs.info(`selected ${f.name} — ${Math.round(selected.image.length / 1024)} KiB`);
		} catch (e) {
			selected = null;
			logs.error(`${f.name}: ${(e as Error).message}`);
		}
	}

	function ask(title: string, confirm: string, get: () => Promise<Uint8Array>) {
		pending = { title, confirm, get };
	}

	async function run() {
		const job = pending;
		pending = null;
		if (!job) return;
		fwup.open(job.title);
		try {
			const image = await job.get();
			// An untagged image still flashes, but the puck it leaves cannot do
			// panel updates -- the next one needs UF2 DFU and drag-and-drop.
			if (!imagePanelUpdatable(image)) {
				fwup.modal.detail =
					'This image has no panel-update support — after it, the next update needs UF2 DFU and drag-and-drop.';
			}
			await fwup.run(device.transportRef, image);
			fwup.finish(true, 'Rebooting to apply. The panel reconnects itself once the puck returns.');
		} catch (e) {
			logs.error(`update failed: ${(e as Error).message}`);
			fwup.finish(false, (e as Error).message);
		}
	}

	const installed = $derived(status?.build.id ?? '');
	const newest = $derived(releases.list?.find((r) => !r.prerelease && relAsset(r, false)) ?? null);
	const updateAvailable = $derived(
		!!newest && !!installed && compareVersions(newest.tag_name, installed) > 0 ? newest : null,
	);

	const fmtDate = (s: string) => new Date(s).toLocaleDateString();

	function flashRelease(rel: Release) {
		const isFactory = !!factory[rel.tag_name];
		const asset = relAsset(rel, isFactory);
		if (!asset) {
			logs.error(`release ${rel.tag_name} has no ${isFactory ? 'factory-reset' : 'standard'} .uf2 asset`);
			return;
		}
		ask(
			`Updating to ${rel.tag_name}${isFactory ? ' (factory reset)' : ''}`,
			isFactory
				? `Flash ${rel.tag_name} factory-reset? This wipes ALL settings and the controller pairing — you must re-pair.`
				: `Update the puck to ${rel.tag_name}?`,
			() => releases.download(asset, (pct) => fwup.stage(`Downloading ${asset.name}`, pct)),
		);
	}

	$effect(() => {
		if (device.connected && !device.isDongle) void releases.load();
	});
</script>

<div class="space-y-4">
	{#if gated}
		<div class="border-error-500 bg-error-500/10 rounded-container border p-4">
			<h2 class="text-error-700-300 flex items-center gap-2 text-sm font-semibold">
				<TriangleAlertIcon size={16} /> Panel updates not supported by this firmware
			</h2>
			<p class="text-app-muted mt-1 text-xs">
				This puck is running {status?.build.id || 'an unknown build'} (status v{status?.protocol ?? '?'}), which
				predates panel updates (needs v15+), so updating from this page is disabled. One manual flash gets you back:
				choose <strong>UF2 DFU</strong> in the Device menu, then drag a panel-update-capable .uf2 onto the UF2BOOT drive it
				mounts. Every update after that happens right here.
			</p>
		</div>
	{/if}

	{#if updateAvailable}
		<div class="border-primary-500 bg-primary-500/10 rounded-container flex items-center gap-3 border p-3">
			<span class="text-sm">
				<strong>{updateAvailable.tag_name}</strong> is available — you have {installed || '—'}.
			</span>
		</div>
	{/if}

	<Panel title="Update from a local file" disabled={gated}>
		{#snippet info()}
			<InfoPopover title="Local file update">
				The image is streamed over WebUSB into spare flash, verified <em>on the puck</em>, and applied on an automatic
				reboot. Nothing is armed until it verifies — a failed or interrupted transfer leaves the running firmware
				untouched, and even a power cut during the apply only leaves the puck in its UF2 bootloader for drag-and-drop
				recovery.
			</InfoPopover>
		{/snippet}

		<FileUpload
			accept=".uf2,application/octet-stream"
			maxFiles={1}
			onFileAccept={(e) => pickFile(e.files[0])}
			onFileReject={() => logs.error('that is not a .uf2 file')}
		>
			<FileUpload.Dropzone
				class="rounded-container border-app-line hover:border-primary-500 data-[dragging]:border-success-500 data-[dragging]:bg-success-500/10 w-full cursor-pointer border-2 border-dashed p-6 text-center transition-colors"
			>
				<div class="flex items-center justify-center gap-2 text-sm font-semibold">
					<UploadIcon size={16} /> Drop a .uf2 file here — or click to browse
				</div>
				<div class="text-app-muted mt-1 text-xs">
					{selected ? `${selected.name} — ${Math.round(selected.image.length / 1024)} KiB` : 'no file selected'}
				</div>
			</FileUpload.Dropzone>
			<FileUpload.HiddenInput />
		</FileUpload>

		<div class="mt-3">
			<button
				type="button"
				class="btn preset-filled-primary-500 btn-sm"
				disabled={!selected || !device.connected}
				onclick={() =>
					selected && ask('Flashing firmware', `Flash ${selected.name} to the puck?`, async () => selected!.image)}
			>
				Flash firmware
			</button>
		</div>
	</Panel>

	{#if !device.isDongle}
		<Panel title="Update from a release" disabled={gated}>
			{#snippet actions()}
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
					onclick={() => releases.load(true)}
				>
					<RefreshCwIcon size={13} /> Refresh
				</button>
			{/snippet}
			{#snippet info()}
				<InfoPopover title="Releases">
					Official builds from github.com/{REL_REPO}/releases. <strong>Factory reset</strong> flashes the
					<code>-factory-reset</code> build of that version: on its first boot it wipes ALL settings and the controller pairing
					(you must re-pair), then behaves like the standard build. Use it to recover from a bad config or stale bond.
				</InfoPopover>
			{/snippet}

			<p class="text-app-muted mb-2 text-xs">
				Installed: <b class="text-secondary-700-300">{installed || '—'}</b>
			</p>

			{#if releases.loading}
				<p class="text-app-muted text-sm">Loading…</p>
			{:else if releases.error}
				<p class="text-app-muted text-sm">
					Could not load releases ({releases.error}) —
					<a
						class="text-primary-700-300 underline"
						href="https://github.com/{REL_REPO}/releases"
						target="_blank"
						rel="noopener">open the releases page</a
					> and use the local-file card.
				</p>
			{:else if releases.list}
				<div class="divide-app-line-soft divide-y">
					{#each releases.list as rel (rel.tag_name)}
						{@const updatable = releases.panelUpdatable(rel)}
						<div class="flex flex-wrap items-center gap-3 py-2.5">
							<span class="w-20 shrink-0 font-semibold">{rel.tag_name}</span>
							<span class="text-app-muted w-24 shrink-0 text-xs">{fmtDate(rel.published_at)}</span>
							{#if rel.prerelease}
								<span
									class="bg-warning-100-900 text-warning-700-300 rounded-full px-2 py-0.5 text-[10px] font-semibold"
								>
									pre-release
								</span>
							{/if}
							{#if updatable === false}
								<span
									class="bg-warning-100-900 text-warning-700-300 rounded-full px-2 py-0.5 text-[10px] font-semibold"
									title="After this build, the next update needs UF2 DFU and drag-and-drop."
								>
									no panel updates
								</span>
							{/if}
							<span class="flex-1"></span>
							<label class="text-app-muted flex shrink-0 items-center gap-1.5 text-xs">
								<input type="checkbox" class="checkbox" bind:checked={factory[rel.tag_name]} />
								Factory reset
							</label>
							<button
								type="button"
								class="btn preset-tonal-surface btn-sm"
								disabled={gated || !device.connected}
								onclick={() => flashRelease(rel)}
							>
								Flash
							</button>
						</div>
						{#if rel.body}
							<details class="pb-2">
								<summary class="text-primary-700-300 cursor-pointer text-xs">Release notes</summary>
								<pre
									class="bg-app-well border-app-line mt-1 max-h-64 overflow-auto rounded-base border p-2 text-xs whitespace-pre-wrap">{rel.body}</pre>
							</details>
						{/if}
					{/each}
				</div>
			{/if}
		</Panel>
	{/if}
</div>

<ConfirmDialog
	spec={pending
		? {
				title: pending.title,
				body: [pending.confirm],
				confirmLabel: 'Flash',
				onConfirm: run,
			}
		: null}
	onCancel={() => (pending = null)}
/>
