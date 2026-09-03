<script lang="ts">
	import type { Snippet } from 'svelte';

	interface Props {
		title: string;
		info?: Snippet;
		actions?: Snippet;
		children: Snippet;
		/** Greys out and blocks the panel, for settings this firmware can't do. */
		disabled?: boolean;
		class?: string;
	}
	let { title, info, actions, children, disabled = false, class: klass = '' }: Props = $props();
</script>

<section
	class="bg-app-card border-app-line rounded-container border p-4 {klass}"
	class:opacity-50={disabled}
	class:pointer-events-none={disabled}
>
	<header class="mb-3 flex items-center gap-2">
		<h2 class="text-app-muted text-xs font-semibold tracking-[0.08em] uppercase">{title}</h2>
		{#if info}{@render info()}{/if}
		{#if actions}<div class="ml-auto flex items-center gap-2">{@render actions()}</div>{/if}
	</header>
	{@render children()}
</section>
