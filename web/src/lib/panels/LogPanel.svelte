<script lang="ts">
	import CopyIcon from '@lucide/svelte/icons/copy';
	import GripIcon from '@lucide/svelte/icons/grip';
	import GripVerticalIcon from '@lucide/svelte/icons/grip-vertical';
	import MaximizeIcon from '@lucide/svelte/icons/maximize';
	import MinimizeIcon from '@lucide/svelte/icons/minimize';
	import MinusIcon from '@lucide/svelte/icons/minus';
	import PauseIcon from '@lucide/svelte/icons/pause';
	import PlayIcon from '@lucide/svelte/icons/play';
	import Trash2Icon from '@lucide/svelte/icons/trash-2';
	import XIcon from '@lucide/svelte/icons/x';
	import { FloatingPanel, Portal } from '@skeletonlabs/skeleton-svelte';
	import { panels } from '$lib/state/panels.svelte';
	import { logs, type LogLevel } from '$lib/state/log.svelte';

	const state = $derived(panels.panels.logs);

	const LEVEL_CLASS: Record<LogLevel, string> = {
		info: 'text-app-muted',
		ok: 'text-success-700-300',
		warn: 'text-warning-700-300',
		error: 'text-error-700-300',
	};

	function copyAll() {
		navigator.clipboard?.writeText(logs.text);
	}
</script>

<FloatingPanel
	open={state.open}
	onOpenChange={(e) => panels.set('logs', { open: e.open })}
	size={state.size}
	onSizeChange={(e) => panels.set('logs', { size: e.size })}
	minSize={{ width: 380, height: 220 }}
>
	<Portal>
		<FloatingPanel.Positioner class="z-40">
			<FloatingPanel.Content
				class="bg-app-card border-app-line rounded-container flex flex-col overflow-hidden border shadow-2xl"
			>
				<FloatingPanel.DragTrigger>
					<FloatingPanel.Header
						class="bg-app-well border-app-line flex cursor-move items-center gap-2 border-b px-3 py-1.5"
					>
						<FloatingPanel.Title class="flex items-center gap-1.5 text-xs font-semibold tracking-wide uppercase">
							<GripVerticalIcon size={13} class="text-app-faint" />
							Logs
						</FloatingPanel.Title>
						<span class="text-app-faint tabular-nums text-[11px]">{logs.entries.length}</span>
						<FloatingPanel.Control class="ml-auto flex items-center gap-1">
							<FloatingPanel.StageTrigger
								stage="minimized"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Minimize"><MinusIcon size={14} /></FloatingPanel.StageTrigger
							>
							<FloatingPanel.StageTrigger
								stage="maximized"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Maximize"><MaximizeIcon size={14} /></FloatingPanel.StageTrigger
							>
							<FloatingPanel.StageTrigger
								stage="default"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Restore"><MinimizeIcon size={14} /></FloatingPanel.StageTrigger
							>
							<FloatingPanel.CloseTrigger class="text-app-muted hover:text-error-700-300 px-1" aria-label="Close"
								><XIcon size={14} /></FloatingPanel.CloseTrigger
							>
						</FloatingPanel.Control>
					</FloatingPanel.Header>
				</FloatingPanel.DragTrigger>

				<div class="border-app-line-soft flex items-center gap-2 border-b px-2 py-1.5">
					<input
						bind:value={logs.filter}
						placeholder="Filter…"
						class="bg-app-well border-app-line rounded-base min-w-0 flex-1 border px-2 py-1 text-xs"
					/>
					<button
						type="button"
						onclick={() => (logs.paused = !logs.paused)}
						class="text-app-muted hover:text-app-strong px-1"
						title={logs.paused ? 'Resume' : 'Pause'}
						aria-label={logs.paused ? 'Resume' : 'Pause'}
					>
						{#if logs.paused}<PlayIcon size={14} />{:else}<PauseIcon size={14} />{/if}
					</button>
					<button
						type="button"
						onclick={copyAll}
						class="text-app-muted hover:text-app-strong px-1"
						title="Copy all"
						aria-label="Copy all"><CopyIcon size={14} /></button
					>
					<button
						type="button"
						onclick={() => logs.clear()}
						class="text-app-muted hover:text-app-strong px-1"
						title="Clear"
						aria-label="Clear"><Trash2Icon size={14} /></button
					>
				</div>

				<FloatingPanel.Body class="min-h-0 flex-1 overflow-auto p-2 font-mono text-[11.5px] leading-relaxed">
					{#if logs.visible.length === 0}
						<p class="text-app-faint">No entries.</p>
					{:else}
						{#each logs.visible as e (e.id)}
							<div class="flex gap-2">
								<span class="text-app-faint tabular-nums shrink-0">{e.at.toLocaleTimeString()}</span>
								<span class="{LEVEL_CLASS[e.level]} break-all whitespace-pre-wrap">{e.message}</span>
							</div>
						{/each}
					{/if}
				</FloatingPanel.Body>

				<FloatingPanel.ResizeTrigger
					axis="se"
					class="text-app-faint hover:text-app-muted absolute right-0 bottom-0 cursor-se-resize p-1"
					aria-label="Resize"><GripIcon size={12} /></FloatingPanel.ResizeTrigger
				>
			</FloatingPanel.Content>
		</FloatingPanel.Positioner>
	</Portal>
</FloatingPanel>
