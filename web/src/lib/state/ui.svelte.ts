// Which section the rail is showing, plus the two visibility tiers the old
// panel drove with .debugonly / .logonly classes and ?debug= / ?beta= flags.
// Kept as the same URL flags so existing links and muscle memory still work.

import ActivityIcon from '@lucide/svelte/icons/activity';
import Gamepad2Icon from '@lucide/svelte/icons/gamepad-2';
import GaugeIcon from '@lucide/svelte/icons/gauge';
import HardDriveDownloadIcon from '@lucide/svelte/icons/hard-drive-download';
import KeyboardIcon from '@lucide/svelte/icons/keyboard';
import RadioIcon from '@lucide/svelte/icons/radio';
import VibrateIcon from '@lucide/svelte/icons/vibrate';
import WaypointsIcon from '@lucide/svelte/icons/waypoints';
import type { Component } from 'svelte';

export type SectionId = 'overview' | 'modes' | 'mapping' | 'feel' | 'desktop' | 'firmware' | 'diagnostics' | 'sniffer';

export interface SectionDef {
	id: SectionId;
	label: string;
	icon: Component;
	/** Only reachable once the Beta warning has been accepted. */
	beta?: boolean;
}

/**
 * Sections a ReversePuck dongle has any use for. It emulates a controller
 * rather than hosting one, so the per-controller config does not apply and the
 * release list is puck firmware -- it flashes from the local-file card.
 */
export const DONGLE_SECTIONS: SectionId[] = ['overview', 'firmware', 'sniffer'];

/** Rail entries, in display order. */
export const SECTIONS: SectionDef[] = [
	{ id: 'overview', label: 'Overview', icon: GaugeIcon },
	{ id: 'modes', label: 'Modes', icon: Gamepad2Icon },
	{ id: 'mapping', label: 'Mapping', icon: WaypointsIcon },
	{ id: 'feel', label: 'Feel', icon: VibrateIcon },
	{ id: 'desktop', label: 'Desktop', icon: KeyboardIcon },
	{ id: 'firmware', label: 'Firmware', icon: HardDriveDownloadIcon, beta: true },
	{ id: 'diagnostics', label: 'Diagnostics', icon: ActivityIcon },
	// The sniffer is a separate board, not the puck, so it keeps its own
	// connection. It stays in the nav either way: you may want to watch RF
	// traffic while configuring a puck in another section.
	{ id: 'sniffer', label: 'RF Sniffer', icon: RadioIcon },
];

function readFlag(name: string): boolean {
	if (typeof location === 'undefined') return false;
	return new URLSearchParams(location.search).get(name) === 'true';
}

const SECTION_IDS = new Set<string>([
	'overview',
	'modes',
	'mapping',
	'feel',
	'desktop',
	'firmware',
	'diagnostics',
	'sniffer',
]);

function sectionFromHash(): SectionId {
	if (typeof location === 'undefined') return 'overview';
	const h = location.hash.replace(/^#/, '');
	return SECTION_IDS.has(h) ? (h as SectionId) : 'overview';
}

class UiState {
	section = $state<SectionId>(sectionFromHash());
	/** ?debug=true -- reveals what the old panel marked .debugonly. */
	debug = $state(readFlag('debug'));
	/** ?beta=true or the Beta button -- unlocks firmware flashing. */
	beta = $state(readFlag('beta'));
	/** Collapses the rail to icons; also forced by the <1280px breakpoint. */
	railCollapsed = $state(false);
	monitorOpen = $state(true);

	go(id: SectionId) {
		this.section = id;
		// Keep the URL addressable without adding a history entry per click.
		if (typeof history !== 'undefined') history.replaceState(null, '', `#${id}`);
	}

	/** Follow back/forward and externally-set hashes. */
	listenToHash() {
		if (typeof window === 'undefined') return;
		window.addEventListener('hashchange', () => (this.section = sectionFromHash()));
	}
}

/** Which section is showing, plus the ?debug / ?beta visibility tiers. */
export const ui = new UiState();
