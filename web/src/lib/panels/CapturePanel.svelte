<script lang="ts">
	import DownloadIcon from '@lucide/svelte/icons/download';
	import PlayIcon from '@lucide/svelte/icons/play';
	import SquareIcon from '@lucide/svelte/icons/square';
	import { device } from '$lib/state/device.svelte';
	import { diag } from '$lib/state/diag.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';
	import StreamPanel from './StreamPanel.svelte';

	function download() {
		const a = document.createElement('a');
		a.href = URL.createObjectURL(new Blob([diag.captureText], { type: 'text/plain' }));
		a.download = 'puck-capture.txt';
		a.click();
		URL.revokeObjectURL(a.href);
	}
</script>

<StreamPanel id="capture" title="Capture" badge={`${diag.capLines.length}`}>
	{#snippet toolbar()}
		<button
			type="button"
			class="btn preset-filled-primary-500 btn-sm flex items-center gap-1.5 text-xs"
			disabled={!device.connected || diag.capturing}
			onclick={() => diag.startCapture()}
		>
			<PlayIcon size={12} /> Dump from boot
		</button>
		<button
			type="button"
			class="btn preset-tonal-surface btn-sm flex items-center gap-1.5 text-xs"
			disabled={!diag.capturing}
			onclick={() => diag.stopCapture()}
		>
			<SquareIcon size={12} /> Stop
		</button>
		<span class="ml-auto">
			<InfoPopover title="Capture — host → controller commands">
				The firmware logs everything from boot into a big RAM ring: Steam's writes (<code>ifN</code>), the frames we
				transmit to the controller (<code>TX→ctlr</code>), and RF <code>LINK UP/DOWN</code> edges. For the reconnect
				buzz: trigger it (it appears moments after the puck boots / the controller reconnects), then connect here and
				click <b>Dump from boot</b> — the trigger is still in the ring. Dump streams the whole ring (oldest→newest) and
				then keeps live-updating until you press Stop. <code>cmd</code> is the command/report byte, then the raw bytes.
				(Only present in a logging build, <code>-DOPK_LOG=1</code>.)
			</InfoPopover>
		</span>
		<button
			type="button"
			class="text-app-muted hover:text-app-strong px-1"
			title="Download .txt"
			aria-label="Download capture"
			disabled={!diag.capLines.length}
			onclick={download}
		>
			<DownloadIcon size={14} />
		</button>
	{/snippet}

	{#if !device.status?.logEnabled}
		<p class="text-app-muted">This firmware is not a logging build (-DOPK_LOG=1), so the capture ring is empty.</p>
	{:else if !diag.capLines.length}
		<p class="text-app-muted">
			(nothing captured yet — press <b>Dump from boot</b>)
		</p>
	{:else}
		<pre class="whitespace-pre">{diag.captureText}</pre>
	{/if}
</StreamPanel>
