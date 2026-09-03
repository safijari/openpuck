// The original panel was dark-only. Dark stays the default -- it is what
// OpenPuck looks like -- but the mode is now the viewer's choice.

export type Mode = 'dark' | 'light';

const KEY = 'opk_theme';

function initial(): Mode {
	try {
		const saved = localStorage.getItem(KEY);
		if (saved === 'light' || saved === 'dark') return saved;
	} catch {
		// Private mode: fall through to the default.
	}
	return 'dark';
}

class ThemeState {
	mode = $state<Mode>(initial());

	constructor() {
		// Skeleton reads color-scheme, which the `dark` class drives via the
		// custom variant in app.css.
		$effect.root(() => {
			$effect(() => {
				document.documentElement.classList.toggle('dark', this.mode === 'dark');
				try {
					localStorage.setItem(KEY, this.mode);
				} catch {
					// Not worth failing the page over.
				}
			});
		});
	}

	toggle() {
		this.mode = this.mode === 'dark' ? 'light' : 'dark';
	}
}

/** Light/dark preference, persisted; dark is the default. */
export const theme = new ThemeState();
