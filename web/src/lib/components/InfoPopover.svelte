<script lang="ts">
	import InfoIcon from '@lucide/svelte/icons/info';
	import { Popover, Portal } from '@skeletonlabs/skeleton-svelte';
	import type { Snippet } from 'svelte';

	interface Props {
		title?: string;
		children: Snippet;
	}
	let { title, children }: Props = $props();
</script>

<!--
	The old panel printed all 27 of these notes as permanent body copy: ~7,460
	characters competing with the controls on every load. The text is unchanged,
	it is just on demand now.
-->
<Popover>
	<Popover.Trigger
		class="text-app-faint hover:text-primary-700-300 inline-flex shrink-0 items-center transition-colors"
		aria-label={title ? `About ${title}` : 'More information'}
	>
		<InfoIcon size={14} />
	</Popover.Trigger>
	<Portal>
		<Popover.Positioner class="z-50">
			<Popover.Content
				class="bg-app-card border-app-line rounded-container max-w-sm border p-3 text-[13px] leading-relaxed shadow-xl"
			>
				{#if title}
					<div class="mb-1 text-xs font-semibold tracking-wide uppercase">{title}</div>
				{/if}
				<div class="text-app-strong">{@render children()}</div>
			</Popover.Content>
		</Popover.Positioner>
	</Portal>
</Popover>
