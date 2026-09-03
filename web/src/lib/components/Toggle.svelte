<script lang="ts">
	import { Switch } from '@skeletonlabs/skeleton-svelte';

	interface Props {
		checked: boolean;
		disabled?: boolean;
		/** Visible label; also names the control for assistive tech. */
		label: string;
		onChange: (next: boolean) => void;
	}
	let { checked, disabled = false, label, onChange }: Props = $props();
</script>

<!--
	A real switch rather than a button reading "on"/"off": that carried no
	aria-checked, so a screen reader announced a button whose only state was the
	word inside it, and it was not operable as a toggle.

	Skeleton's Switch is headless, so the track and thumb are styled here; zag
	sets data-state on both, which is what the checked colours key off.
-->
<Switch
	{checked}
	{disabled}
	onCheckedChange={(e) => onChange(e.checked)}
	class="flex items-center gap-2.5 {disabled ? 'opacity-50' : 'cursor-pointer'}"
>
	<Switch.Control
		class="border-app-line bg-app-well data-[state=checked]:bg-success-500 data-[state=checked]:border-success-500
			focus-within:ring-primary-500 inline-flex h-5 w-9 shrink-0 items-center rounded-full border p-0.5
			transition-colors focus-within:ring-2"
	>
		<Switch.Thumb
			class="bg-app-muted data-[state=checked]:bg-success-contrast-500 size-3.5 rounded-full transition-transform
				data-[state=checked]:translate-x-4"
		/>
	</Switch.Control>
	<Switch.Label class="text-sm">{label}</Switch.Label>
	<Switch.HiddenInput />
</Switch>
