<script lang="ts">
	interface Props {
		label: string;
		value?: string | number | null;
		/** Renders the value as a coloured pill instead of plain text. */
		tone?: 'none' | 'up' | 'down' | 'warn';
		mono?: boolean;
		hint?: string;
	}
	let { label, value, tone = 'none', mono = false, hint }: Props = $props();

	const shown = $derived(value === null || value === undefined || value === '' ? '—' : String(value));
	const pill = {
		up: 'bg-success-100-900 text-success-700-300',
		down: 'bg-error-100-900 text-error-700-300',
		warn: 'bg-warning-100-900 text-warning-700-300',
	} as const;
</script>

<div class="bg-app-well border-app-line-soft rounded-base border p-2.5" title={hint}>
	<div class="text-app-muted text-[11px] font-medium tracking-wider uppercase">{label}</div>
	{#if tone === 'none'}
		<div class="tabular-nums mt-0.5 text-lg {mono ? 'font-mono text-sm' : ''}">{shown}</div>
	{:else}
		<div class="mt-1">
			<span class="rounded-full px-2 py-0.5 text-xs font-semibold {pill[tone]}">{shown}</span>
		</div>
	{/if}
</div>
