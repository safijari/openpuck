import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import tailwindcss from '@tailwindcss/vite';
import { VitePWA } from 'vite-plugin-pwa';
import { resolve } from 'node:path';

export default defineConfig({
	// Tailwind must come before the Svelte plugin.
	plugins: [
		tailwindcss(),
		svelte(),
		VitePWA({
			// Relative scope, because Pages serves this from /openpuck/ rather
			// than a domain root.
			base: './',
			scope: './',
			registerType: 'autoUpdate',
			includeAssets: ['icon.svg', 'apple-touch-icon.png', 'sniffer.html'],
			manifest: {
				name: 'OpenPuck Config',
				short_name: 'OpenPuck',
				description: 'Configure and flash an OpenPuck over WebUSB.',
				start_url: './index.html',
				scope: './',
				display: 'standalone',
				orientation: 'any',
				background_color: '#11131a',
				theme_color: '#11131a',
				icons: [
					{ src: './icon-192.png', sizes: '192x192', type: 'image/png' },
					{ src: './icon-512.png', sizes: '512x512', type: 'image/png' },
					{ src: './icon-maskable-512.png', sizes: '512x512', type: 'image/png', purpose: 'maskable' },
				],
			},
			workbox: {
				globPatterns: ['**/*.{js,css,html,svg,png,woff2}'],
				// legacy.html is the previous hand-written panel kept for
				// side-by-side comparison, not part of the app -- precaching it
				// would put 165 KiB of dead weight in every install.
				globIgnores: ['legacy.html', 'legacy-sniffer.html'],
				// The panel is the whole app: precaching it means it opens and can
				// talk to a puck with no network at all. Only the GitHub release
				// list needs one, and that failing is already handled.
				navigateFallback: './index.html',
				// The fallback would otherwise serve the app for any navigation
				// under the scope, which makes legacy.html unreachable the moment
				// the worker installs -- and the whole point of keeping it is
				// being able to open it side by side against the same puck.
				navigateFallbackDenylist: [/\.md$/, /legacy(-sniffer)?\.html$/],
				cleanupOutdatedCaches: true,
			},
			devOptions: { enabled: false },
		}),
	],
	resolve: { alias: { $lib: resolve(import.meta.dirname, 'src/lib') } },
	build: {
		// Pages serves docs/ straight from the branch, and that directory also
		// holds hand-authored markdown (PROTOCOL.md, BUILD_AND_DEPLOY.md,
		// WEBUSB_LINUX.md, TESTING_GUIDE.md) plus .nojekyll -- emptyOutDir
		// would delete them.
		outDir: '../docs',
		emptyOutDir: false,
		rollupOptions: {
			// sniffer.html is a redirect stub, not an app entry: the sniffer is a
			// section of the panel now. It is copied verbatim so the old URL,
			// linked from puck_sniffer/README.md, keeps working.
			input: { index: resolve(import.meta.dirname, 'index.html') },
		},
	},
	// Relative asset URLs so the same build works at /openpuck/ on Pages and at
	// the server root when previewing locally.
	base: './',
});
