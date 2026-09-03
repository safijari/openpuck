<script lang="ts">
	interface Props {
		/** Whether the connected firmware supports the setting. */
		ok: boolean | undefined;
		/** Protocol version that introduced it, e.g. "v19". */
		requires: string;
	}
	let { ok, requires }: Props = $props();
</script>

<!--
	The original rendered a whole ~100px card per unsupported setting whose only
	content was "update to configure this". A disabled control plus this badge
	says the same thing without the real estate.
-->
{#if ok === false}
	<span
		class="bg-warning-100-900 text-warning-700-300 shrink-0 rounded px-1.5 py-0.5 text-[10px] font-semibold whitespace-nowrap"
		title="This puck's firmware predates the setting — update it to configure it."
	>
		needs fw ≥ {requires}
	</span>
{/if}
