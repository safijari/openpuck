<script lang="ts">
	import CheckIcon from '@lucide/svelte/icons/check';
	import MenuIcon from '@lucide/svelte/icons/menu';
	import ChevronDownIcon from '@lucide/svelte/icons/chevron-down';
	import MoonIcon from '@lucide/svelte/icons/moon';
	import PanelsTopLeftIcon from '@lucide/svelte/icons/panels-top-left';
	import PlugZapIcon from '@lucide/svelte/icons/plug-zap';
	import SunIcon from '@lucide/svelte/icons/sun';
	import UsbIcon from '@lucide/svelte/icons/usb';
	import { Menu, Portal } from '@skeletonlabs/skeleton-svelte';
	import { device } from '$lib/state/device.svelte';
	import { DONGLE_SECTIONS, SECTIONS, ui } from '$lib/state/ui.svelte';
	import { panels, type PanelId } from '$lib/state/panels.svelte';
	import { theme } from '$lib/state/theme.svelte';
	import { DEBUG_CDC_FIELD, FACTORY_ERASE_CMD, OP, WIPE_BOARD_CMD } from '$lib/protocol/fields';
	import { logs } from '$lib/state/log.svelte';
	import ConfirmDialog, { type ConfirmSpec } from '$lib/components/ConfirmDialog.svelte';

	const status = $derived(device.status);

	const CONN_LABEL: Record<string, string> = {
		disconnected: 'disconnected',
		connecting: 'connecting…',
		connected: 'connected',
		lost: 'device lost',
	};

	let confirming = $state<ConfirmSpec | null>(null);

	// Everything that reboots, erases, or re-enumerates the puck. These sat as
	// six always-clickable buttons in the old top bar, Factory erase and Wipe
	// board included, right next to Connect. Each now needs a confirmation, and
	// the two irreversible ones need the exact word typed.
	const DEVICE_ACTIONS: (Omit<ConfirmSpec, 'onConfirm'> & {
		id: string;
		label: string;
		debugOnly?: boolean;
		puckOnly?: boolean;
		run: () => Promise<void>;
	})[] = [
		{
			id: 'debugCdc',
			label: 'Debug CDC',
			title: 'Reboot with the CDC serial console?',
			body: [
				'The puck reboots and comes back with a serial port (115200 baud) instead of WebUSB — this panel will disconnect and NOT reconnect until the next reboot.',
				'Connect a serial monitor to capture logs, then replug (or reboot) to return to normal WebUSB mode. The debug console auto-reverts after one boot.',
			],
			confirmLabel: 'Reboot',
			run: async () => {
				await device.setField(DEBUG_CDC_FIELD, 1);
				logs.info('debug-CDC reboot sent — device disconnecting; reconnect a serial monitor at 115200 baud');
			},
		},
		{
			id: 'dfuSerial',
			label: 'Serial DFU',
			title: 'Reboot into serial DFU?',
			body: ['The puck will disconnect immediately. Flash with adafruit-nrfutil, then replug.'],
			confirmLabel: 'Reboot',
			run: async () => {
				await device.sendRaw([OP.dfuSerial]);
				logs.info('serial DFU reboot sent — device disconnecting');
			},
		},
		{
			id: 'dfuUf2',
			label: 'UF2 DFU',
			title: 'Reboot into the UF2 bootloader?',
			body: ['The puck will disconnect and mount as a USB drive. Drag the .uf2 file onto it to flash.'],
			confirmLabel: 'Reboot',
			run: async () => {
				await device.sendRaw([OP.dfuUf2]);
				logs.info('UF2 bootloader reboot sent — device disconnecting');
			},
		},
		{
			id: 'factoryErase',
			label: 'Factory erase',
			puckOnly: true,
			danger: true,
			title: 'Factory erase?',
			body: ['This wipes ALL persistent storage on the copycat:'],
			bullets: [
				"the paired-controller bond (you'll have to re-pair)",
				'every saved setting (mode, chords, back paddles, sensitivity)',
			],
			typeToConfirm: 'ERASE',
			confirmLabel: 'Erase everything',
			run: async () => {
				await device.sendRaw(FACTORY_ERASE_CMD);
				logs.warn(
					'FACTORY ERASE sent — copycat is reformatting and rebooting to defaults. Re-pair the controller, then reconnect.',
				);
			},
		},
		{
			id: 'wipeBoard',
			label: 'Wipe board',
			danger: true,
			debugOnly: true,
			title: 'Wipe the entire board?',
			body: ['This is NOT a factory reset. It erases:'],
			bullets: [
				'the OpenPuck FIRMWARE itself',
				'every setting (mode, chords, back paddles, sensitivity)',
				'the paired-controller bond',
			],
			typeToConfirm: 'WIPE',
			confirmLabel: 'Wipe the board',
			run: async () => {
				await device.sendRaw(WIPE_BOARD_CMD);
				logs.warn(
					'FULL BOARD WIPE sent — the board is erasing firmware + all data (~15-20 s), then reboots as a blank UF2 drive. Flash OpenPuck (.uf2) to restore it.',
				);
			},
		},
	];

	function pick(a: (typeof DEVICE_ACTIONS)[number]) {
		if (!device.connected) {
			logs.warn('not connected');
			return;
		}
		const { id, label, debugOnly, puckOnly, run, ...spec } = a;
		confirming = { ...spec, onConfirm: () => void run() };
	}

	const PANEL_LABELS: Record<PanelId, string> = { logs: 'Logs', capture: 'Capture', trail: 'Loop trail' };
</script>

<header
	class="border-app-line bg-app-chrome flex shrink-0 items-center gap-2 overflow-x-auto border-b px-3 py-2 whitespace-nowrap sm:gap-3 sm:px-4"
>
	<!-- Below lg the layout stacks and the rail is hidden, so navigation lives
	     here instead. -->
	<Menu>
		<Menu.Trigger class="btn preset-tonal-surface flex items-center gap-1 px-2 text-sm lg:hidden" aria-label="Sections">
			<MenuIcon size={16} />
			<ChevronDownIcon size={12} />
		</Menu.Trigger>
		<Portal>
			<Menu.Positioner class="z-50">
				<Menu.Content class="bg-app-card border-app-line rounded-container min-w-44 border p-1 shadow-xl">
					{#each SECTIONS.filter((x) => !device.isDongle || DONGLE_SECTIONS.includes(x.id)) as x (x.id)}
						{@const Icon = x.icon}
						<Menu.Item
							value={x.id}
							onclick={() => ui.go(x.id)}
							class="rounded-base hover:bg-app-hover flex cursor-pointer items-center gap-2 px-2.5 py-1.5 text-sm
								{ui.section === x.id ? 'text-primary-700-300' : ''}"
						>
							<Icon size={15} />
							{x.label}
						</Menu.Item>
					{/each}
				</Menu.Content>
			</Menu.Positioner>
		</Portal>
	</Menu>

	<span class="shrink-0 font-bold tracking-tight">OpenPuck</span>

	{#if device.demo}
		<span class="bg-warning-100-900 text-warning-700-300 shrink-0 rounded-full px-2 py-0.5 text-xs font-semibold">
			demo data<span class="hidden sm:inline">&nbsp;— no device</span>
		</span>
	{:else}
		<span
			class="shrink-0 rounded-full px-2 py-0.5 text-xs font-semibold
			{device.connected ? 'bg-success-100-900 text-success-700-300' : 'bg-error-100-900 text-error-700-300'}"
		>
			{CONN_LABEL[device.conn]}
		</span>
	{/if}

	{#if device.isDongle}
		<span class="text-app-strong rounded-base bg-app-well border-app-line border px-2 py-0.5 text-xs">
			ReversePuck
		</span>
		{#if device.serial}<span class="text-app-muted text-xs">{device.serial}</span>{/if}
	{:else if status}
		<span
			class="text-app-strong rounded-base bg-app-well border-app-line hidden shrink-0 border px-2 py-0.5 text-xs sm:inline"
		>
			{status.modeName}
		</span>
		<span class="text-app-muted hidden text-xs sm:inline">fw {status.build.id || '—'}</span>
	{/if}

	<div class="flex-1"></div>

	<Menu>
		<Menu.Trigger class="btn preset-tonal-surface flex items-center gap-1.5 text-sm">
			<PanelsTopLeftIcon size={15} />
			<span class="hidden lg:inline">Panels</span>
			<ChevronDownIcon size={13} />
		</Menu.Trigger>
		<Portal>
			<Menu.Positioner class="z-50">
				<Menu.Content class="bg-app-card border-app-line rounded-container min-w-44 border p-1 shadow-xl">
					{#each Object.entries(PANEL_LABELS) as [id, label] (id)}
						<Menu.Item
							value={id}
							onclick={() => panels.toggle(id as PanelId)}
							class="hover:bg-app-hover rounded-base flex cursor-pointer items-center gap-2 px-2.5 py-1.5 text-sm"
						>
							<span class="text-primary-700-300 flex w-3.5 justify-center">
								{#if panels.panels[id as PanelId].open}<CheckIcon size={14} />{/if}
							</span>
							{label}
						</Menu.Item>
					{/each}
				</Menu.Content>
			</Menu.Positioner>
		</Portal>
	</Menu>

	<button
		type="button"
		onclick={() => theme.toggle()}
		class="btn preset-tonal-surface text-sm"
		aria-label="Toggle colour scheme"
		title="Toggle light / dark"
	>
		{#if theme.mode === 'dark'}
			<MoonIcon size={15} />
		{:else}
			<SunIcon size={15} />
		{/if}
	</button>

	<button
		type="button"
		onclick={() => (ui.beta = !ui.beta)}
		class="btn text-sm {ui.beta ? 'preset-filled-warning-500' : 'preset-tonal-warning'}"
	>
		Beta {ui.beta ? 'on' : ''}
	</button>

	<Menu>
		<Menu.Trigger class="btn preset-tonal-surface flex items-center gap-1.5 text-sm">
			<UsbIcon size={15} />
			<span class="hidden lg:inline">Device</span>
			<ChevronDownIcon size={13} />
		</Menu.Trigger>
		<Portal>
			<Menu.Positioner class="z-50">
				<Menu.Content class="bg-app-card border-app-line rounded-container min-w-52 border p-1 shadow-xl">
					{#each DEVICE_ACTIONS as a (a.id)}
						{#if (!a.debugOnly || ui.debug) && !(a.puckOnly && device.isDongle)}
							<Menu.Item
								value={a.id}
								onclick={() => pick(a)}
								class="rounded-base cursor-pointer px-2.5 py-1.5 text-sm
									{a.danger ? 'text-error-700-300 hover:bg-error-500/15' : 'hover:bg-app-hover'}
									{device.connected ? '' : 'pointer-events-none opacity-40'}"
							>
								{a.label}
							</Menu.Item>
						{/if}
					{/each}
				</Menu.Content>
			</Menu.Positioner>
		</Portal>
	</Menu>

	<button
		type="button"
		class="btn preset-filled-primary-500 flex items-center gap-1.5 text-sm"
		onclick={() => device.connect()}
	>
		<PlugZapIcon size={15} />
		<span class="hidden sm:inline">{device.connected ? 'Reconnect' : 'Connect'}</span>
	</button>
</header>

<ConfirmDialog spec={confirming} onCancel={() => (confirming = null)} />
