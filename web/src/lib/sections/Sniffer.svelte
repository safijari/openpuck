<script lang="ts">
	import DownloadIcon from '@lucide/svelte/icons/download';
	import PlugZapIcon from '@lucide/svelte/icons/plug-zap';
	import Trash2Icon from '@lucide/svelte/icons/trash-2';
	import { sniffer } from '$lib/state/sniffer.svelte';
	import { hex, parsePinSession } from '$lib/protocol/sniffer';
	import { logs } from '$lib/state/log.svelte';
	import Panel from '$lib/components/Panel.svelte';
	import Stat from '$lib/components/Stat.svelte';

	const s = $derived(sniffer.status);

	let tgtIbex = $state('');
	let pinCh = $state('');
	let pinSess = $state('');

	// The filter object is replaced rather than mutated so the derived row list
	// recomputes; re-filtering is cheap next to the capture rate.
	function refilter() {
		sniffer.applyFilters();
	}
</script>

<!--
	The sniffer is a different board from the puck (VID 0x28DE only), so it owns
	its own connection rather than sharing the panel's. Both can be connected at
	once, which is the point: watch RF traffic while changing a puck setting.
-->
<div class="mb-4 flex flex-wrap items-center gap-3">
	<span
		class="rounded-full px-2 py-0.5 text-xs font-semibold
		{sniffer.connected ? 'bg-success-100-900 text-success-700-300' : 'bg-error-100-900 text-error-700-300'}"
	>
		{sniffer.connected ? 'sniffer connected' : 'sniffer not connected'}
	</span>
	{#if s}
		<span class="text-xs font-semibold {s.capturing ? 'text-success-700-300' : 'text-warning-700-300'}">
			{s.capturing
				? s.camped
					? 'CAPTURE (auto-camped on learned bond)'
					: 'CAPTURE (session locked)'
				: 'ACQUIRE (scanning ibex/ch2)'}
		</span>
	{/if}
	<span class="flex-1"></span>
	<button
		type="button"
		class="btn preset-filled-primary-500 btn-sm flex items-center gap-1.5"
		onclick={() => sniffer.connect()}
	>
		<PlugZapIcon size={14} />
		{sniffer.connected ? 'Reconnect sniffer' : 'Connect sniffer'}
	</button>
</div>
<div class="space-y-4">
	<div class="grid gap-2 autofit-[150px]">
		<Stat label="P→C" value={sniffer.stats.pc} />
		<Stat label="C→P" value={sniffer.stats.cp} />
		<Stat label="Bad CRC" value={sniffer.stats.bad} tone={sniffer.stats.bad ? 'down' : 'none'} />
		<Stat label="Recorded" value={sniffer.stats.recorded} />
		<Stat
			label="Device drops"
			value={s?.drops ?? null}
			tone={s && s.drops ? 'down' : 'none'}
			hint="Frames lost to a full ring on the device. 0 = lossless."
		/>
		<Stat
			label="Bonds"
			value={s ? (s.bondCount ? `${s.bondCount} learned` : 'none yet') : null}
			tone={s?.camped ? 'up' : 'none'}
		/>
	</div>

	<Panel title="Session">
		<div class="grid gap-3 text-sm autofit-[280px]">
			<div>
				<div class="text-app-faint text-[10px] font-semibold tracking-wider uppercase">Locked session</div>
				<div class="font-mono text-xs">
					{s
						? `ch ${s.channel}  addr ${hex(s.base)}/${s.prefix.toString(16).padStart(2, '0')}  (last pipe ${s.lastPipe})`
						: '—'}
				</div>
			</div>
			<div>
				<div class="text-app-faint text-[10px] font-semibold tracking-wider uppercase">Advertised (E1)</div>
				<div class="font-mono text-xs">
					{s && s.advChannel
						? `ch ${s.advChannel}  addr ${hex(s.advBase)}/${s.advPrefix.toString(16).padStart(2, '0')}`
						: '— (no E1 yet)'}
				</div>
			</div>
		</div>

		<div class="mt-3 flex flex-wrap items-center gap-2">
			<button
				type="button"
				class="btn btn-sm {sniffer.capturing ? 'preset-filled-error-500' : 'preset-filled-primary-500'}"
				disabled={!sniffer.connected}
				onclick={() => sniffer.toggleCapture()}
			>
				{sniffer.capturing ? '■ Stop Cap' : '● Start Cap'}
			</button>
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm"
				disabled={!sniffer.connected}
				onclick={() => sniffer.reacquire()}
			>
				Re-acquire
			</button>
			<button
				type="button"
				class="btn btn-sm {sniffer.surveying ? 'preset-filled-warning-500' : 'preset-tonal-surface'}"
				disabled={!sniffer.connected}
				onclick={() => sniffer.toggleSurvey()}
			>
				Survey
			</button>
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm"
				disabled={!sniffer.connected}
				onclick={() => sniffer.forgetBonds()}
			>
				Forget bonds
			</button>
			<span class="flex-1"></span>
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
				onclick={() => sniffer.downloadJson()}
			>
				<DownloadIcon size={13} /> JSON
			</button>
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
				onclick={() => sniffer.downloadText()}
			>
				<DownloadIcon size={13} /> Text
			</button>
			<button
				type="button"
				class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
				onclick={() => sniffer.clear()}
			>
				<Trash2Icon size={13} /> Clear
			</button>
		</div>

		<div class="mt-3 grid gap-2 autofit-[260px]">
			<label class="flex items-center gap-2 text-xs">
				<span class="text-app-muted w-24 shrink-0">Lock ibex</span>
				<input
					bind:value={tgtIbex}
					placeholder="hex"
					class="input bg-app-well border-app-line min-w-0 flex-1 border px-2 py-1 font-mono"
				/>
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm"
					disabled={!sniffer.connected}
					onclick={() => {
						const v = parseInt(tgtIbex.trim().replace(/^0x/i, ''), 16);
						if (Number.isFinite(v)) sniffer.setTargetIbex(v);
						else logs.error('lock ibex: not a hex byte');
					}}>Set</button
				>
			</label>
			<label class="flex items-center gap-2 text-xs">
				<span class="text-app-muted w-24 shrink-0">Pin channel</span>
				<input
					bind:value={pinCh}
					placeholder="0-83"
					class="input bg-app-well border-app-line min-w-0 flex-1 border px-2 py-1 font-mono"
				/>
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm"
					disabled={!sniffer.connected}
					onclick={() => {
						const c = parseInt(pinCh, 10);
						if (Number.isFinite(c)) sniffer.pinChannel(c);
						else logs.error('pin channel: not a number');
					}}>Set</button
				>
			</label>
			<label class="flex items-center gap-2 text-xs">
				<span class="text-app-muted w-24 shrink-0">Pin session</span>
				<input
					bind:value={pinSess}
					placeholder="b0 b1 b2 b3 pfx ch"
					class="input bg-app-well border-app-line min-w-0 flex-1 border px-2 py-1 font-mono"
				/>
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm"
					disabled={!sniffer.connected}
					onclick={() => {
						const v = parsePinSession(pinSess);
						if (v) sniffer.pinSession(v);
						else logs.error('pin session: expected "b0 b1 b2 b3 pfx ch"');
					}}>Set</button
				>
			</label>
		</div>
	</Panel>

	<Panel title="Frames">
		{#snippet actions()}
			<span class="text-app-muted text-xs tabular-nums">showing {sniffer.stats.shown}</span>
		{/snippet}

		<div class="mb-3 flex flex-wrap items-center gap-3 text-xs">
			<label class="flex items-center gap-1.5">
				<input type="checkbox" class="checkbox" bind:checked={sniffer.filter.hideRoutine} onchange={refilter} />
				Hide routine polls
			</label>
			<select class="select w-32 text-xs" bind:value={sniffer.filter.dir} onchange={refilter}>
				<option value="">any direction</option>
				<option value="P→C">P→C</option>
				<option value="C→P">C→P</option>
			</select>
			<input
				class="input bg-app-well border-app-line w-24 border px-2 py-1 font-mono"
				placeholder="opcode"
				bind:value={sniffer.filter.op}
				oninput={refilter}
			/>
			<select class="select w-36 text-xs" bind:value={sniffer.filter.len} onchange={refilter}>
				<option value="">any length</option>
				<option value="ne49">not 49</option>
				<option value="gt49">&gt; 49</option>
				<option value="gt1">&gt; 1</option>
			</select>
			<input
				class="input bg-app-well border-app-line min-w-40 flex-1 border px-2 py-1 font-mono"
				placeholder="hex match in payload"
				bind:value={sniffer.filter.hexMatch}
				oninput={refilter}
			/>
		</div>

		<div class="overflow-x-auto">
			<table class="w-full text-left font-mono text-[11px]">
				<thead class="text-app-faint border-app-line border-b">
					<tr>
						{#each ['#', 'ms', 'dir', 'ch', 'pipe', 'op', 'crc', 'rssi', 'len', 'bytes'] as h (h)}
							<th class="py-1 pr-2 font-medium whitespace-nowrap">{h}</th>
						{/each}
					</tr>
				</thead>
				<tbody>
					{#each sniffer.rows as e (e.seq)}
						<tr class="border-app-line-soft border-b" class:opacity-60={!e.crc}>
							<td class="py-0.5 pr-2 tabular-nums">{e.seq}</td>
							<td class="py-0.5 pr-2 tabular-nums">{e.ms.toFixed(1)}</td>
							<td class="py-0.5 pr-2 {e.dir === 'P→C' ? 'text-primary-700-300' : 'text-secondary-700-300'}">{e.dir}</td>
							<td class="py-0.5 pr-2 tabular-nums">{e.ch}</td>
							<td class="py-0.5 pr-2 tabular-nums">{e.pipe}</td>
							<td class="py-0.5 pr-2">{e.op}</td>
							<td class="py-0.5 pr-2" class:text-error-700-300={!e.crc}>{e.crc ? 'ok' : 'BAD'}</td>
							<td class="py-0.5 pr-2 tabular-nums">{e.rssi}</td>
							<td class="py-0.5 pr-2 tabular-nums">{e.len}</td>
							<td class="text-app-muted py-0.5 break-all">{e.raw}</td>
						</tr>
					{/each}
				</tbody>
			</table>
			{#if !sniffer.rows.length}
				<p class="text-app-muted py-6 text-center text-sm">
					{sniffer.connected ? 'No frames match the current filters.' : 'Connect the sniffer board to start.'}
				</p>
			{/if}
		</div>
	</Panel>
</div>
