// Floating panels for the live streams. The old panel put these at the bottom
// of a 4000px scroll (#log rendered 34px tall, 3880px down the page), which
// made them unusable while actually operating the device.

export type PanelId = 'logs' | 'capture' | 'trail';

export interface PanelState {
	open: boolean;
	size: { width: number; height: number };
	position: { x: number; y: number };
}

const DEFAULTS: Record<PanelId, PanelState> = {
	logs: { open: false, size: { width: 520, height: 300 }, position: { x: 80, y: 120 } },
	capture: { open: false, size: { width: 680, height: 340 }, position: { x: 120, y: 160 } },
	trail: { open: false, size: { width: 620, height: 320 }, position: { x: 160, y: 200 } },
};

const KEY = 'opk_panels_v1';

function load(): Record<PanelId, PanelState> {
	const base = structuredClone(DEFAULTS);
	try {
		const raw = localStorage.getItem(KEY);
		if (!raw) return base;
		const saved = JSON.parse(raw) as Partial<Record<PanelId, PanelState>>;
		for (const id of Object.keys(base) as PanelId[]) {
			if (saved[id]) Object.assign(base[id], saved[id]);
		}
	} catch {
		// A corrupt or unavailable store must never stop the panel loading.
	}
	return base;
}

class PanelStore {
	panels = $state(load());

	toggle(id: PanelId) {
		this.panels[id].open = !this.panels[id].open;
		this.persist();
	}

	set(id: PanelId, patch: Partial<PanelState>) {
		Object.assign(this.panels[id], patch);
		this.persist();
	}

	get anyOpen() {
		return Object.values(this.panels).some((p) => p.open);
	}

	private persist() {
		try {
			localStorage.setItem(KEY, JSON.stringify(this.panels));
		} catch {
			// Private-mode / quota. Losing panel geometry is not worth an error.
		}
	}
}

/** Open state and geometry of the floating panels, persisted per browser. */
export const panels = new PanelStore();
