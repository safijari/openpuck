// Loop-state trail: every change of the loop pill, timestamped. Persisted to
// localStorage so an unattended hang is still recorded after a page refresh --
// the original kept this under the same key, so existing trails survive.

const KEY = 'opk_loop_trail';
const CAP = 400;

export interface TrailEntry {
	t: number;
	msg: string;
}

function load(): TrailEntry[] {
	try {
		const raw = localStorage.getItem(KEY);
		return raw ? (JSON.parse(raw) as TrailEntry[]) : [];
	} catch {
		return [];
	}
}

class TrailStore {
	entries = $state<TrailEntry[]>(load());

	add(msg: string) {
		this.entries.unshift({ t: Date.now(), msg });
		if (this.entries.length > CAP) this.entries.length = CAP;
		this.persist();
	}

	clear() {
		this.entries = [];
		this.persist();
	}

	private persist() {
		try {
			localStorage.setItem(KEY, JSON.stringify(this.entries));
		} catch {
			// Quota or private mode; the in-memory trail still works.
		}
	}
}

/** Loop-state trail, persisted so an unattended hang survives a refresh. */
export const trail = new TrailStore();
