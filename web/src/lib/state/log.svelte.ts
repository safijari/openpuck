// Replaces the old log(): a single div whose entire contents were rebuilt as
// `newest + "\n" + previous` then truncated with .slice(0, 2000) -- which cut
// mid-line and silently dropped history (docs/index.html:547).

export type LogLevel = 'info' | 'warn' | 'error' | 'ok';

export interface LogEntry {
	id: number;
	at: Date;
	level: LogLevel;
	message: string;
}

/** Entries kept in memory. Old panel effectively held ~25 lines. */
const CAP = 2000;

class LogStore {
	entries = $state<LogEntry[]>([]);
	/** Freezes rendering while you read; new entries still accumulate. */
	paused = $state(false);
	filter = $state('');
	private seq = 0;

	push(message: string, level: LogLevel = 'info') {
		this.entries.push({ id: this.seq++, at: new Date(), level, message });
		if (this.entries.length > CAP) this.entries.splice(0, this.entries.length - CAP);
	}

	info = (m: string) => this.push(m, 'info');
	warn = (m: string) => this.push(m, 'warn');
	error = (m: string) => this.push(m, 'error');
	ok = (m: string) => this.push(m, 'ok');

	clear() {
		this.entries = [];
	}

	/** Newest first, matching the reading order the old panel used. */
	get visible() {
		const q = this.filter.trim().toLowerCase();
		const rows = q ? this.entries.filter((e) => e.message.toLowerCase().includes(q)) : this.entries;
		return rows.slice().reverse();
	}

	get text() {
		return this.entries.map((e) => `${e.at.toLocaleTimeString()}  ${e.message}`).join('\n');
	}
}

/** The activity log behind the floating Logs panel. */
export const logs = new LogStore();
