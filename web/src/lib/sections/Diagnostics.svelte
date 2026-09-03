<script lang="ts">
	import DownloadIcon from '@lucide/svelte/icons/download';
	import RefreshCwIcon from '@lucide/svelte/icons/refresh-cw';
	import Trash2Icon from '@lucide/svelte/icons/trash-2';
	import { device } from '$lib/state/device.svelte';
	import { diag, fmtDuration } from '$lib/state/diag.svelte';
	import { formatFlight, formatWedgeVitals } from '$lib/protocol/flight';
	import { OP } from '$lib/protocol/fields';
	import { logs } from '$lib/state/log.svelte';
	import { ui } from '$lib/state/ui.svelte';
	import Panel from '$lib/components/Panel.svelte';
	import Stat from '$lib/components/Stat.svelte';
	import InfoPopover from '$lib/components/InfoPopover.svelte';

	const status = $derived(device.status);
	const hex = (v: number) => '0x' + v.toString(16).padStart(8, '0');

	function downloadCsv() {
		const a = document.createElement('a');
		a.href = URL.createObjectURL(new Blob([diag.hangCsv], { type: 'text/csv' }));
		a.download = 'openpuck-hang-log.csv';
		a.click();
		URL.revokeObjectURL(a.href);
	}

	// Least-ever free stack on the usbd task. Low means overflow risk; the old
	// panel flagged red under 24 words (~96B) and amber under 48.
	const usbdTone = $derived.by(() => {
		const w = status?.loop.usbdStackWords;
		if (w == null) return 'none' as const;
		return w < 24 ? ('down' as const) : w < 48 ? ('warn' as const) : ('none' as const);
	});
</script>

<div class="space-y-4">
	<Panel title="Global telemetry">
		{#snippet info()}
			<InfoPopover title="Global telemetry">
				Sums over all controllers; the sidebar shows the selected controller's own rates. Polls/s ≈ 250 expected; it's
				capped by Loop period (≈4000 µs needed). "Slowest stage" is the loop section eating the most time per iteration.
				Poll TX and RF-fail counts show even when the link reads down, so a wedge stays diagnosable: polls above zero
				while Delivered reads "—" means the puck is still polling but getting no usable replies.
			</InfoPopover>
		{/snippet}
		<div class="grid gap-2 autofit-[150px]">
			<Stat label="Delivered (all)" value={status?.link.up ? `${status.rates.delivered} /s` : null} />
			<Stat label="New reports (all)" value={status?.link.up ? `${status.rates.newReports} /s` : null} />
			<Stat label="Polls/s (all)" value={status ? `${status.rates.polls} /s` : null} />
			<Stat label="Relay/s (all)" value={status?.rates.relay != null ? `${status.rates.relay} /s` : null} />
			<Stat
				label="RF fails/s"
				value={status ? `${status.rates.crc} · ${status.rates.norx} · ${status.rates.heal}` : null}
				hint="crc · noRx · heal"
			/>
			<Stat
				label="Clock LF / HF"
				value={status?.clock ? `${status.clock.lf} / ${status.clock.hf}` : null}
				tone={status?.clock && (status.clock.lfBad || status.clock.hfBad) ? 'down' : 'none'}
			/>
			<Stat
				label="µs per ms"
				value={status?.clock?.usPerMs || null}
				tone={status?.clock?.usPerMsBad ? 'down' : 'none'}
				hint="ideal 1000"
			/>
			<Stat label="usbd stack free" value={status?.loop.usbdStackWords} tone={usbdTone} hint="words" />
			<Stat label="Loop period" value={status?.loop.periodUs ? `${status.loop.periodUs} µs` : null} />
			<Stat
				label="Slowest stage"
				value={status?.loop.periodUs ? `${status.loop.worstStage} ${status.loop.worstUs}µs` : null}
			/>
			<Stat
				label="Poll period act/want"
				value={status?.link.up ? `${status.loop.pollUs} / ${status.loop.pollIntendedUs} µs` : null}
			/>
			<Stat
				label="Ring faults"
				value={status?.rates.ringFault || null}
				tone={status?.rates.ringFault ? 'down' : 'none'}
			/>
		</div>
	</Panel>

	<Panel title="Reset &amp; fault history">
		{#snippet info()}
			<InfoPopover title="Reset detail">
				<em>watchdog (hang)</em>/<em>CPU lockup</em>/<em>HARDFAULT</em> = a fault worth reporting (issue #72);
				<em>reboot</em> = an intentional reset (mode change / config); <em>power-on</em> and <em>pin/replug</em> = normal
				plug-in.
			</InfoPopover>
		{/snippet}
		{#if status?.reset}
			<div class="grid gap-2 autofit-[150px]">
				<Stat label="Last reset" value={status.reset.name} tone={status.reset.isFault ? 'down' : 'none'} />
				<Stat label="Hung in" value={status.reset.hangStageName} />
				<Stat label="Hang PC" value={status.reset.hangPC ? hex(status.reset.hangPC) : null} mono />
				<Stat label="Hang LR" value={status.reset.hangLR ? hex(status.reset.hangLR) : null} mono />
			</div>
			<p class="text-app-muted mt-2 font-mono text-xs">RESETREAS={hex(status.reset.raw)}</p>
		{:else}
			<p class="text-app-muted text-sm">Not reported by this firmware.</p>
		{/if}
	</Panel>

	<Panel title="IMU (raw)">
		{#snippet info()}
			<InfoPopover title="IMU (raw)">
				Raw SC2 accel (before scaling). This controller is a ±2 g sensor (~16384 = 1 g); the Switch report divides it by
				4 to present the genuine ±8 g (4096 = 1 g) scale so the console's gravity-correction engages and the gyro stops
				drifting.
			</InfoPopover>
		{/snippet}
		{#if status?.imu}
			<p class="font-mono text-sm">
				a=({status.imu.ax}, {status.imu.ay}, {status.imu.az}) |a|={status.imu.magnitude}
			</p>
		{:else}
			<p class="text-app-muted text-sm">—</p>
		{/if}
	</Panel>

	<Panel title="Controller actions">
		<div class="space-y-2">
			<div class="flex flex-wrap items-center gap-3">
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm w-44 shrink-0"
					disabled={!device.connected}
					onclick={async () => {
						await device.sendRaw([OP.controllerOff]);
						logs.info('controller power-off attempt sent — watch link status');
					}}
				>
					Turn off controller
				</button>
				<span class="text-app-muted flex-1 text-xs">
					Fires the controller power-off (0x9F "off!", x3) — the same path Steam's "turn off controller" and
					host-suspend use.
				</span>
			</div>

			{#if ui.debug}
				<div class="flex flex-wrap items-center gap-3">
					<button
						type="button"
						class="btn preset-tonal-surface btn-sm w-44 shrink-0"
						disabled={!device.connected}
						onclick={async () => {
							await device.sendRaw([OP.hapticClear]);
							logs.info('haptic re-init sent (clear stuck buzz)');
						}}
					>
						Clear stuck buzz
					</button>
					<span class="text-app-muted flex-1 text-xs">
						If the controller's haptics get stuck/buzzing, this re-inits them (same as pressing Steam).
					</span>
				</div>

				<div class="flex flex-wrap items-center gap-3">
					<button
						type="button"
						class="btn btn-sm w-44 shrink-0 {diag.stabArmed ? 'preset-filled-warning-500' : 'preset-tonal-surface'}"
						disabled={!device.connected}
						onclick={() => diag.toggleStability()}
					>
						{diag.stabArmed ? 'Stop stability test' : 'Test stability'}
					</button>
					<span class="text-app-muted flex-1 text-xs">
						Buzzes the controllers every 10 s (keeps them awake) and times how long the puck stays up. Runs are kept in
						this tab; a refresh clears them.
					</span>
				</div>
				{#if diag.stabRuns.length}
					<p class="text-app-muted pl-1 text-xs">
						Runs: {diag.stabRuns.map((s) => fmtDuration(s)).join(' · ')}
					</p>
				{/if}
			{/if}
		</div>
	</Panel>

	{#if ui.debug}
		<Panel title="Hang log">
			{#snippet info()}
				<InfoPopover title="Hang log">
					One row per reset: uptime before it, reason, the stuck loop stage and captured PC (when the watchdog ISR could
					run), and usbd stack free. Accumulates across resets in this tab — a page refresh clears it. Pair with <em
						>Test stability</em
					> for unattended runs.
				</InfoPopover>
			{/snippet}
			{#snippet actions()}
				<button
					type="button"
					class="text-app-muted hover:text-app-strong px-1"
					title="Download CSV"
					aria-label="Download CSV"
					disabled={!diag.hangLog.length}
					onclick={downloadCsv}><DownloadIcon size={14} /></button
				>
				<button
					type="button"
					class="text-app-muted hover:text-error-700-300 px-1"
					title="Clear"
					aria-label="Clear hang log"
					disabled={!diag.hangLog.length}
					onclick={() => diag.clearHangLog()}><Trash2Icon size={14} /></button
				>
			{/snippet}

			{#if !diag.hangLog.length}
				<p class="text-app-muted text-sm">No resets logged yet.</p>
			{:else}
				<div class="overflow-x-auto">
					<table class="w-full text-left font-mono text-xs">
						<thead class="text-app-faint border-app-line border-b">
							<tr>
								<th class="py-1 pr-3 font-medium">Time</th>
								<th class="py-1 pr-3 font-medium">Uptime</th>
								<th class="py-1 pr-3 font-medium">Reason</th>
								<th class="py-1 pr-3 font-medium">Stage</th>
								<th class="py-1 pr-3 font-medium">PC</th>
								<th class="py-1 pr-3 font-medium">usbd</th>
							</tr>
						</thead>
						<tbody>
							{#each diag.hangLog as r, i (i)}
								<tr class="border-app-line-soft border-b">
									<td class="py-1 pr-3">{r.time}</td>
									<td class="py-1 pr-3">{r.uptime != null ? fmtDuration(r.uptime) : '—'}</td>
									<td class="py-1 pr-3">{r.reason}</td>
									<td class="py-1 pr-3">{r.stage || '—'}</td>
									<td class="py-1 pr-3">{r.pc || '—'}</td>
									<td class="py-1 pr-3">{r.usbd ?? '—'}</td>
								</tr>
							{/each}
						</tbody>
					</table>
				</div>
			{/if}
		</Panel>

		<Panel title="Flight recorder">
			{#snippet info()}
				<InfoPopover title="Flight recorder">
					The trail of events the firmware recorded in the seconds before its most recent <b>watchdog hang</b> — the
					ring survives the reset in <code>.noinit</code> RAM (like the hang PC), so it's the post-mortem of what wedged
					the board. Loads automatically after a hang reconnects. Same data the console <code>FR</code>
					command prints.
				</InfoPopover>
			{/snippet}
			{#snippet actions()}
				<button
					type="button"
					class="btn preset-tonal-surface btn-sm flex items-center gap-1.5"
					disabled={!device.connected || diag.flightLoading}
					onclick={() => diag.loadFlight()}
				>
					<RefreshCwIcon size={13} />
					{diag.flightLoading ? 'Loading…' : 'Load last-hang trail'}
				</button>
			{/snippet}

			{#if diag.flightHeader}
				<p class="text-app-muted mb-2 font-mono text-xs">
					<b>@wedge:</b>
					{formatWedgeVitals(diag.flightHeader)}
				</p>
				<p class="text-app-muted mb-2 text-xs">
					Showing {diag.flightEvents.length} of {diag.flightHeader.total} events before the last hang.
				</p>
				<pre
					class="bg-app-well border-app-line rounded-base max-h-80 overflow-auto border p-2 font-mono text-xs whitespace-pre">{formatFlight(
						diag.flightEvents,
					)}</pre>
			{:else}
				<p class="text-app-muted text-sm">
					No trail loaded — the last boot wasn't a hang, or the recorder didn't survive this board's reset.
				</p>
			{/if}
		</Panel>
	{/if}
</div>
