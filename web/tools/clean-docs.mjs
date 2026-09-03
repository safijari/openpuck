// Removes the generated assets from docs/ before a build.
//
// The build cannot use emptyOutDir: docs/ also holds hand-authored markdown and
// .nojekyll that Pages serves, plus the legacy panel. But without a clean the
// hashed chunks from every previous build linger, and since docs/ is committed
// that means the repo grows by a bundle every time anyone rebuilds.

import { existsSync, readdirSync, rmSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const docs = fileURLToPath(new URL('../../docs', import.meta.url));

// Everything the build emits. Workbox's runtime is content-hashed, so without
// this every build would leave the previous one behind.
const GENERATED = /^(sw\.js|registerSW\.js|manifest\.webmanifest|workbox-[\w-]+\.js)$/;

const assets = `${docs}/assets`;
if (existsSync(assets)) rmSync(assets, { recursive: true, force: true });

let n = 0;
for (const f of readdirSync(docs)) {
	if (GENERATED.test(f)) {
		rmSync(`${docs}/${f}`, { force: true });
		n++;
	}
}
console.log(`cleaned docs/assets and ${n} generated file(s)`);
