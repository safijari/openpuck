# OpenPuck web panel

The WebUSB configurator served at
[safijari.github.io/openpuck](https://safijari.github.io/openpuck/). This
directory is the source; **it builds into `../docs/`**, which is what GitHub
Pages serves.

This document assumes you know C and can read JavaScript, but have not used
Svelte or Skeleton. It is written for someone arriving from the firmware side of
this repo.

---

## Quick start

```sh
cd web
npm install
npm run dev        # http://localhost:5173, reloads as you edit
```

WebUSB needs a secure context; `http://localhost` counts, `file://` does not.

No hardware? `?fixture=true` renders the whole panel against a recorded status
blob, and `?fixture=dongle` renders the ReversePuck layout. Both label
themselves "demo data" so recorded values are never mistaken for a real puck.

| Command          | What it does                                 |
| ---------------- | -------------------------------------------- |
| `npm run dev`    | Dev server with hot reload                   |
| `npm run build`  | Regenerate `../docs/`                        |
| `npm test`       | Run the protocol decoder tests               |
| `npm run check`  | Type-check (`svelte-check`)                  |
| `npm run format` | Format everything (prettier)                 |
| `npm run theme`  | Regenerate the colour theme from its palette |

Before pushing: `npm test && npm run check && npm run format:check`, then
`npm run build` and commit the regenerated `docs/` in the same commit. CI checks
all of that, including that `docs/` matches the sources.

> **If a section goes blank while developing**, restart the dev server before
> suspecting your code. A stale hot-reload state can blank a section with no
> error, and it is very convincing.

---

## What Svelte is, in five minutes

Svelte is a compiler. A `.svelte` file is one component: markup, its script, and
its behaviour together. There is no virtual DOM and no render loop — the
compiler works out which DOM nodes depend on which values and writes direct
updates.

A component looks like this:

```svelte
<script lang="ts">
	let count = $state(0); // reactive value
	const doubled = $derived(count * 2); // recomputes when count changes
</script>

<button onclick={() => count++}>
	{count} doubled is {doubled}
</button>
```

Three things to know:

**`$state`, `$derived`, `$effect` are "runes".** They are compiler
instructions, not function calls — you cannot store `$state` in a variable or
pass it around. `$state(x)` marks a value as reactive; anything reading it
re-runs when it changes. `$derived(expr)` is a value computed from others.
`$effect(() => ...)` runs a side effect when its dependencies change.

**`$effect` is a trap for newcomers, including me.** An effect that reads and
writes the same state loops forever and freezes the tab. If you find yourself
using an effect to compute a value, you want `$derived`. Use `$effect` only for
genuine side effects — talking to the DOM, starting a timer. One-shot setup at
startup belongs in plain top-level code, not an effect (see `src/App.svelte`).

**Markup blocks** replace the template loops you would write by hand:

```svelte
{#if connected}
	...
{:else}
	...
{/if}
{#each modes as m (m.id)}
	...
{/each}
<!-- (m.id) is the key -->
{#await promise then value} ... {/await}
```

Anything in `{ }` is an expression. `class={...}` and `onclick={...}` are just
attributes taking values.

### Reactivity outside components

State shared across the app lives in `.svelte.ts` files — a plain module that
may use runes. That is why `src/lib/state/*.svelte.ts` has that odd double
extension. Each exports a single instance:

```ts
class UiState {
	section = $state<SectionId>('overview');
	go(id: SectionId) {
		this.section = id;
	}
}
export const ui = new UiState();
```

Any component reading `ui.section` re-renders when it changes. No subscriptions,
no boilerplate.

---

## What Skeleton is

[Skeleton](https://www.skeleton.dev) is a component library built on Tailwind.
Two halves matter here:

**Components** (`@skeletonlabs/skeleton-svelte`) are _headless_: they provide
behaviour, keyboard handling and ARIA, but almost no styling. You supply classes
to every part. That is why a switch looks like this:

```svelte
<Switch {checked} onCheckedChange={(e) => onChange(e.checked)}>
	<Switch.Control class="h-5 w-9 rounded-full ...">
		<Switch.Thumb class="size-3.5 rounded-full ..." />
	</Switch.Control>
	<Switch.Label>Rumble</Switch.Label>
	<Switch.HiddenInput />
</Switch>
```

Verbose, but you get a real focusable checkbox with `aria-labelledby` instead of
a `<button>` that merely says "on". **Prefer a Skeleton component over
hand-rolling one.** This port learned that twice: a hand-written focus trap lost
a fight with Skeleton's own menu and froze the renderer, and four controls
shipped without the semantics the library would have given for free.

Components use a consistent shape: pass state in as a prop, receive changes
through a callback (`checked` + `onCheckedChange`, `open` + `onOpenChange`).

**Theme** (`@skeletonlabs/skeleton`) supplies colour tokens — `primary`,
`surface`, `error` and so on, each a 50–950 ramp. Skeleton resolves light and
dark through CSS `light-dark()`, which keys off `color-scheme` rather than a
class; `src/app.css` points Tailwind's `dark` variant at a root class so the
whole system flips together, portalled content included.

---

## Styling

All styling is Tailwind utility classes in the markup. There are no `<style>`
blocks and no inline `style=` attributes anywhere, and it should stay that way.

Use the app's semantic tokens rather than raw Skeleton ramps, so light and dark
both work without `dark:` variants everywhere:

| Token                                                   | Use for                            |
| ------------------------------------------------------- | ---------------------------------- |
| `bg-app-bg`                                             | The page                           |
| `bg-app-chrome`                                         | App bar, rail, monitor sidebar     |
| `bg-app-card`                                           | Cards and dialogs                  |
| `bg-app-well`                                           | Recessed areas: stat tiles, inputs |
| `border-app-line` / `border-app-line-soft`              | Borders, dividers                  |
| `text-app-strong` / `text-app-muted` / `text-app-faint` | Text emphasis                      |

For coloured text on a tinted background use the paired ramps —
`text-error-700-300` is the dark shade in light mode and the light shade in dark
mode. A bare `text-error-300` will be unreadable in one of them.

Two custom utilities cover the responsive card grids: `autofit-[320px]` and
`autofill-[230px]`.

---

## How the code is arranged

Strictly layered; nothing imports upward, and CI has no way to check that, so
keep it honest:

```
protocol/  →  usb/  →  state/  →  components, sections, panels  →  App
```

### `src/lib/protocol/` — the important part

Pure wire logic. No DOM, no Svelte, no USB: byte offsets in, typed objects out.
This is where the reverse-engineered knowledge lives, it is the only layer under
test, and **it is the layer to be careful in.**

| File                                | Holds                                                        |
| ----------------------------------- | ------------------------------------------------------------ |
| `blob.ts`                           | The 0xA5 status blob: `parseBlob()` and the capability gates |
| `lizard.ts`                         | The desktop binding map, mirroring `lizard_map.h`            |
| `firmware.ts`                       | UF2 parsing, CRC32, the update protocol                      |
| `backup.ts`                         | The backup/clone file format                                 |
| `capture.ts`, `flight.ts`           | The two diagnostic streams                                   |
| `sniffer.ts`                        | The RF sniffer's frames and filters                          |
| `fields.ts`, `types.ts`, `modes.ts` | Wire field ids and per-controller tables                     |

The comments here explain _why_ an offset or guard exists, and several are
load-bearing — a digital lizard binding with no trigger fires forever; a status
blob truncated mid-transfer must be dropped whole rather than applied. Keep
them.

Adding a firmware field? Decode it in `parseBlob`, gate it on the protocol
version the same way the neighbours are gated, add a test, then surface it in a
section.

### `src/lib/usb/transport.ts`

WebUSB: device selection, the framed reader, the firmware-update endpoint
discipline. It reports through callbacks and a `TransportLogger` interface
rather than importing UI state, which is what keeps the layering honest.

### `src/lib/state/*.svelte.ts`

The stores. `device` owns the connection, the 600 ms poll loop and every device
write; the rest are smaller (`ui`, `logs`, `trail`, `diag`, `fwup`, `releases`,
`sniffer`, `panels`, `theme`).

### `src/lib/sections/`, `components/`, `panels/`

One component per rail section, shared building blocks, and the draggable
floating panels for the live streams.

---

## Common changes

**Add a config setting.** Decode it in `parseBlob` (gated on its protocol
version), add its field id to `fields.ts`, add a test, then add the control to a
section with `device.setField(FIELD.yourThing, value)`.

**Add a section.** Add it to `SECTIONS` in `state/ui.svelte.ts`, create the
component in `sections/`, and register it in `App.svelte` — in `EAGER` for the
common path, or `LAZY` if it is large and rarely opened.

**Change a colour.** Edit the palette in `tools/gen-theme.mjs` and run
`npm run theme`. Never hand-edit `src/openpuck-theme.css`.

---

## Testing

`npm test` runs Vitest over the protocol layer — 116 tests, no browser and no
hardware. Everything touching real bytes should have one: a firmware bug found
by a test is far cheaper than one found by a bricked puck.

The layers above are not unit-tested. They need a real device, and
`docs/legacy.html` (the previous hand-written panel) is kept for one release so
a suspicious reading can be compared side by side against the same puck.
