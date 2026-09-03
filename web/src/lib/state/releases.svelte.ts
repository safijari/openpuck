// GitHub releases for the Firmware section.
//
// Binaries come from the orphan `firmware` branch mirror rather than the
// release asset itself: GitHub's asset CDN sends no CORS headers, so an
// in-page fetch of a release asset fails. release.yml mirrors every UF2 plus a
// manifest.json to that branch (see docs/BUILD_AND_DEPLOY.md:246-250).

import { uf2ToImage } from '$lib/protocol/firmware';

/** Upstream repo the release list and firmware mirror come from. */
export const REL_REPO = 'safijari/openpuck';
const MIRROR = `https://raw.githubusercontent.com/${REL_REPO}/firmware`;

export interface ReleaseAsset {
	name: string;
	size: number;
	url: string;
	browser_download_url: string;
}

export interface Release {
	tag_name: string;
	name: string;
	body: string;
	draft: boolean;
	prerelease: boolean;
	published_at: string;
	assets: ReleaseAsset[];
}

/** name -> { panelUpdate }, from the mirror branch. */
type Manifest = Record<string, { panelUpdate?: boolean }>;

/** Pick a release's standard or factory-reset UF2. */
export function relAsset(rel: Release, factory: boolean): ReleaseAsset | null {
	const suffix = factory ? 'factory-reset' : 'standard';
	return (
		rel.assets.find((a) => a.name === `OpenPuck-${rel.tag_name}-${suffix}.uf2`) ??
		rel.assets.find((a) => new RegExp(`^OpenPuck-.*-${suffix}\\.uf2$`).test(a.name)) ??
		null
	);
}

class ReleaseStore {
	list = $state<Release[] | null>(null);
	manifest = $state<Manifest | null>(null);
	loading = $state(false);
	error = $state<string | null>(null);

	/**
	 * Whether flashing this release leaves a puck that can still do panel
	 * updates. null = not mirrored yet, so don't badge it either way; the mirror
	 * step lags a fresh release by about a minute.
	 */
	panelUpdatable(rel: Release): boolean | null {
		if (!this.manifest) return null;
		const known = [relAsset(rel, false), relAsset(rel, true)].filter((a) => a && this.manifest![a.name]);
		if (!known.length) return null;
		return known.some((a) => this.manifest![a!.name].panelUpdate);
	}

	async load(force = false) {
		if (this.list && !force) return;
		this.loading = true;
		this.error = null;
		try {
			// no-store: the pre-release flag and the notes are edited on GitHub
			// after the fact, and a cached list would keep showing stale state.
			const [r, mr] = await Promise.all([
				fetch(`https://api.github.com/repos/${REL_REPO}/releases?per_page=15`, {
					cache: 'no-store',
					headers: { Accept: 'application/vnd.github+json' },
				}),
				fetch(`${MIRROR}/manifest.json`, { cache: 'no-store' }).catch(() => null),
			]);
			if (!r.ok) throw new Error(`GitHub API ${r.status}`);
			try {
				this.manifest = mr && mr.ok ? ((await mr.json()) as Manifest) : null;
			} catch {
				this.manifest = null;
			}
			this.list = ((await r.json()) as Release[]).filter((x) => !x.draft);
		} catch (e) {
			this.error = (e as Error).message;
			this.list = null;
		} finally {
			this.loading = false;
		}
	}

	/**
	 * Fetch a release image, preferring the CORS-capable mirror. If the mirror
	 * has not caught up, fall back to the API asset URL, and only then hand the
	 * download to a new tab -- the page cannot read it, but the user can still
	 * drop the file on the local-file card.
	 */
	async download(asset: ReleaseAsset, onProgress: (pct: number) => void): Promise<Uint8Array> {
		let resp: Response | null = null;
		try {
			const r = await fetch(`${MIRROR}/${asset.name}`, { cache: 'no-store' });
			if (r.ok) resp = r;
		} catch {
			// Mirror miss; try the asset URL next.
		}
		if (!resp) {
			try {
				const r = await fetch(asset.url, { headers: { Accept: 'application/octet-stream' } });
				if (r.ok) resp = r;
			} catch {
				// Both blocked.
			}
		}
		if (!resp || !resp.body) {
			window.open(asset.browser_download_url, '_blank', 'noopener');
			throw new Error(
				`${asset.name} isn't on the firmware mirror yet and GitHub's asset CDN blocks in-page downloads. ` +
					"It's downloading in a new tab instead — drop the file on the local-file card above.",
			);
		}

		const total = Number(resp.headers.get('content-length')) || asset.size || 0;
		const reader = resp.body.getReader();
		const parts: Uint8Array[] = [];
		let got = 0;
		for (;;) {
			const { done, value } = await reader.read();
			if (done) break;
			parts.push(value);
			got += value.length;
			if (total) onProgress(Math.min(99, Math.floor((got * 100) / total)));
		}
		const buf = new Uint8Array(got);
		let o = 0;
		for (const p of parts) {
			buf.set(p, o);
			o += p.length;
		}
		return uf2ToImage(buf.buffer);
	}
}

/** GitHub releases and the mirrored firmware images. */
export const releases = new ReleaseStore();
