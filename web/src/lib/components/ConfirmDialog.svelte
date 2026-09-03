<script lang="ts">
	import TriangleAlertIcon from '@lucide/svelte/icons/triangle-alert';
	import { Dialog, Portal } from '@skeletonlabs/skeleton-svelte';

	export interface ConfirmSpec {
		title: string;
		/** Paragraphs of explanation; each renders on its own line. */
		body: string[];
		/** Bulleted consequences, for the destructive actions. */
		bullets?: string[];
		confirmLabel: string;
		danger?: boolean;
		/**
		 * When set, the confirm button stays disabled until this exact word is
		 * typed. The original used prompt(); a dialog keeps the browser
		 * responsive, which matters because a native prompt blocks the WebUSB
		 * session underneath it.
		 */
		typeToConfirm?: string;
		onConfirm: () => void;
	}

	interface Props {
		spec: ConfirmSpec | null;
		onCancel: () => void;
	}
	let { spec, onCancel }: Props = $props();

	let typed = $state('');
	let input = $state<HTMLInputElement | null>(null);

	// Reset the box whenever a different action opens the dialog.
	$effect(() => {
		if (spec) typed = '';
	});

	const ready = $derived(!spec?.typeToConfirm || typed === spec.typeToConfirm);
</script>

<!--
	Skeleton's Dialog rather than a hand-rolled overlay: it traps focus, restores
	it on close and blocks the page behind, and being the same focus manager as
	the menus these open from, it does not fight them for focus.
-->
<Dialog
	open={!!spec}
	onOpenChange={(e) => {
		// Guarded: the dialog reports open:false while already closed, and
		// calling onCancel unconditionally there re-renders and re-fires it,
		// which locks the page up.
		if (!e.open && spec) onCancel();
	}}
	role={spec?.danger ? 'alertdialog' : 'dialog'}
	initialFocusEl={() => input}
>
	<Portal>
		<Dialog.Backdrop class="fixed inset-0 z-100 bg-black/70" />
		<Dialog.Positioner class="fixed inset-0 z-100 flex items-center justify-center p-4">
			<Dialog.Content class="bg-app-card border-app-line rounded-container w-[min(520px,94vw)] border p-5 shadow-2xl">
				{#if spec}
					<Dialog.Title
						class="mb-2 flex items-center gap-2 text-base font-semibold {spec.danger ? 'text-error-700-300' : ''}"
					>
						{#if spec.danger}<TriangleAlertIcon size={17} />{/if}
						{spec.title}
					</Dialog.Title>

					<Dialog.Description class="text-app-muted space-y-2 text-sm">
						{#each spec.body as line (line)}
							<p>{line}</p>
						{/each}
					</Dialog.Description>

					{#if spec.bullets?.length}
						<ul class="text-app-muted mt-2 list-disc space-y-0.5 pl-5 text-sm">
							{#each spec.bullets as b (b)}
								<li>{b}</li>
							{/each}
						</ul>
					{/if}

					{#if spec.typeToConfirm}
						<label class="mt-4 block">
							<span class="text-app-muted text-xs">
								Type <strong class="text-error-700-300 font-mono">{spec.typeToConfirm}</strong> to confirm:
							</span>
							<input
								bind:this={input}
								bind:value={typed}
								class="input bg-app-well border-app-line mt-1 w-full border px-2 py-1 font-mono text-sm"
								autocomplete="off"
								spellcheck="false"
							/>
						</label>
					{/if}

					<div class="mt-5 flex justify-end gap-2">
						<Dialog.CloseTrigger class="btn preset-tonal-surface btn-sm">Cancel</Dialog.CloseTrigger>
						<button
							type="button"
							class="btn btn-sm {spec.danger ? 'preset-filled-error-500' : 'preset-filled-primary-500'}"
							disabled={!ready}
							onclick={() => {
								spec.onConfirm();
								onCancel();
							}}
						>
							{spec.confirmLabel}
						</button>
					</div>
				{/if}
			</Dialog.Content>
		</Dialog.Positioner>
	</Portal>
</Dialog>
