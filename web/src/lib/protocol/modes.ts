// Catalogue for the USB-mode picker. The old panel showed 11 unlabelled
// buttons and put the explanation in two dense paragraphs underneath; the
// per-mode text below is that copy, split back onto the mode it describes.

export interface ModeDef {
	id: number;
	name: string;
	summary: string;
	/** Long-form note, shown in the card's info popover. */
	detail?: string;
	tags: string[];
}

/** Every USB mode the panel can switch to, in the order they are shown. */
export const MODES: ModeDef[] = [
	{ id: 0, name: 'Steam (puck)', summary: 'Emulates the official puck for Steam Input.', tags: ['default'] },
	{
		id: 1,
		name: 'Xbox 360',
		summary: 'XInput gamepad; right trackpad drives the mouse.',
		tags: ['rumble', 'trackpad'],
	},
	{ id: 2, name: 'Switch (HORIPAD)', summary: 'Basic Switch pad, no gyro or haptics.', tags: ['PC only'] },
	{
		id: 3,
		name: 'Lizard (always)',
		summary: 'Keyboard, mouse and media output, with no host software.',
		detail:
			'Works on UAC prompts, Task Manager and anywhere else a game pad would not. Configure the bindings in Desktop.',
		tags: ['desktop'],
	},
	{
		id: 4,
		name: 'Switch Pro + gyro',
		summary: 'Full Pro Controller with gyro and haptics.',
		tags: ['gyro', 'rumble', 'console'],
	},
	{ id: 5, name: 'PS5 DualSense', summary: 'DualSense with gyro and trackpad.', tags: ['gyro', 'trackpad', 'PC only'] },
	{
		id: 6,
		name: 'HID gyro (DS4)',
		summary: 'DS4-style report with gyro and trackpad.',
		tags: ['gyro', 'trackpad', 'PC only'],
	},
	{
		id: 9,
		name: 'PS3 (DualShock 3)',
		summary: 'Enumerates on a real PS3, with gyro and haptics.',
		tags: ['gyro', 'rumble', 'console'],
	},
	{
		id: 10,
		name: 'Original Xbox',
		summary: 'One Microsoft Controller S for a real Original Xbox.',
		detail:
			'Uses the Xbox mapping tab; LB/RB become White/Black. A PC needs a driver for the original Xbox controller to see it as a gamepad.',
		tags: ['console'],
	},
	{
		id: 11,
		name: 'DirectInput (sims)',
		summary: 'Every analog input live at once, as two joysticks.',
		detail:
			'For flight/space sims: every analog input is live at once, split over two joysticks (#1 sticks + triggers ' +
			'+ 26 buttons, #2 trackpads + gyro). Trackpad axes hold their last position and re-centre on a pad click. ' +
			'No remapping is applied in this mode — every physical button, paddles included, is its own bindable ' +
			'button, since the sim does the binding. The mode is input-only: DirectInput force feedback is a separate ' +
			'HID class, so rumble is not wired up here.',
		tags: ['gyro', 'trackpad', 'sims'],
	},
	{
		id: 12,
		name: 'SInput (SDL native)',
		summary: 'Open SDL-native protocol; everything at once.',
		detail:
			'The open SDL-native protocol — SDL3 / Steam Input read sticks, both analog triggers, gyro, both trackpads ' +
			'and battery from it natively, with rumble back from the host.',
		tags: ['gyro', 'trackpad', 'rumble'],
	},
];

/** Modes with no WebUSB interface -- reachable only by a back4 chord. */
export const CHORD_ONLY_MODES = [7, 8];
