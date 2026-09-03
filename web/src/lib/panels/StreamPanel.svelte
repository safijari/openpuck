<script lang="ts">
	import GripIcon from '@lucide/svelte/icons/grip';
	import GripVerticalIcon from '@lucide/svelte/icons/grip-vertical';
	import MaximizeIcon from '@lucide/svelte/icons/maximize';
	import MinimizeIcon from '@lucide/svelte/icons/minimize';
	import MinusIcon from '@lucide/svelte/icons/minus';
	import XIcon from '@lucide/svelte/icons/x';
	import { FloatingPanel, Portal } from '@skeletonlabs/skeleton-svelte';
	import { panels, type PanelId } from '$lib/state/panels.svelte';
	import type { Snippet } from 'svelte';

	interface Props {
		id: PanelId;
		title: string;
		/** Shown next to the title, e.g. an entry count. */
		badge?: string;
		toolbar?: Snippet;
		children: Snippet;
	}
	let { id, title, badge, toolbar, children }: Props = $props();

	const state = $derived(panels.panels[id]);
</script>

<FloatingPanel
	open={state.open}
	onOpenChange={(e) => panels.set(id, { open: e.open })}
	size={state.size}
	onSizeChange={(e) => panels.set(id, { size: e.size })}
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
							{title}
						</FloatingPanel.Title>
						{#if badge}<span class="text-app-faint tabular-nums text-[11px]">{badge}</span>{/if}
						<FloatingPanel.Control class="ml-auto flex items-center gap-1">
							<FloatingPanel.StageTrigger
								stage="minimized"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Minimize"
							>
								<MinusIcon size={14} />
							</FloatingPanel.StageTrigger>
							<FloatingPanel.StageTrigger
								stage="maximized"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Maximize"
							>
								<MaximizeIcon size={14} />
							</FloatingPanel.StageTrigger>
							<FloatingPanel.StageTrigger
								stage="default"
								class="text-app-muted hover:text-app-strong px-1"
								aria-label="Restore"
							>
								<MinimizeIcon size={14} />
							</FloatingPanel.StageTrigger>
							<FloatingPanel.CloseTrigger class="text-app-muted hover:text-error-700-300 px-1" aria-label="Close">
								<XIcon size={14} />
							</FloatingPanel.CloseTrigger>
						</FloatingPanel.Control>
					</FloatingPanel.Header>
				</FloatingPanel.DragTrigger>

				{#if toolbar}
					<div class="border-app-line-soft flex items-center gap-2 border-b px-2 py-1.5">{@render toolbar()}</div>
				{/if}

				<FloatingPanel.Body class="min-h-0 flex-1 overflow-auto p-2 font-mono text-[11.5px] leading-relaxed">
					{@render children()}
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
