// Backup / clone: every setting plus the controller bonds, as JSON.
//
// Restoring onto a second puck makes it a clone -- any controller paired to the
// original connects to it with no re-pairing. From docs/index.html:1246-1399.

import { LZ_MAX, type LizardBinding } from './lizard';
import { TYPE_DEFS } from './types';

/** Identifies a backup file; anything else is rejected on import. */
export const BACKUP_MAGIC = 'openpuck-backup';

export interface BackupBond {
	slot: number;
	used: boolean;
	/** 24-byte bond record, hex-encoded. */
	rec: string;
}

export interface BackupTypeCfg {
	back: number[];
	qam: number;
	abSwap: number;
	pad: number;
	led: number;
	rumble?: number;
	padStick?: number[];
}

export interface BackupConfig {
	mode: number;
	mDiv: number;
	mFric: number;
	persistMode: number;
	chord: number[];
	types: BackupTypeCfg[];
	chordD?: number[];
	swGyroLegacy?: number;
	lizardMap?: LizardBinding[];
}

export interface Backup {
	magic: string;
	version: number;
	bonds: BackupBond[];
	config: BackupConfig;
}

export const hexEnc = (arr: ArrayLike<number>) =>
	[...Array.from(arr)].map((x) => x.toString(16).padStart(2, '0')).join('');

export function hexDec(h: string): number[] {
	const a: number[] = [];
	if (!h) return a;
	for (let i = 0; i + 1 < h.length; i += 2) a.push(parseInt(h.substr(i, 2), 16));
	return a;
}

/** Bond dump payload: [count][mask][4 x 24-byte record]. */
export const BOND_FRAME_MIN = 2 + 4 * 24;

/**
 * Build the backup object from a status blob and a bond dump.
 *
 * Settings that a given firmware revision does not have are omitted entirely
 * rather than written as defaults: replaying a default over a newer puck's real
 * setting would silently change it during what the user thinks is a restore.
 */
export function buildBackup(p: Uint8Array, bp: Uint8Array, lizardMap: LizardBinding[] | null): Backup {
	const mask = bp[1];
	const bonds: BackupBond[] = [];
	for (let s = 0; s < 4; s++) {
		bonds.push({ slot: s, used: !!((mask >> s) & 1), rec: hexEnc(bp.slice(2 + s * 24, 2 + s * 24 + 24)) });
	}

	const types: BackupTypeCfg[] = [];
	for (let et = 0; et < TYPE_DEFS.length; et++) {
		const q = 73 + et * 9;
		const t: BackupTypeCfg = {
			back: [p[q], p[q + 1], p[q + 2], p[q + 3]],
			qam: p[q + 4],
			abSwap: p[q + 5],
			pad: p[q + 6],
			led: p[q + 7],
			rumble: p[q + 8],
		};
		// Pad->stick only exists from protocol v20.
		if (p[0] >= 21 && p.length > 192) t.padStick = [p[185 + et * 2], p[186 + et * 2]];
		types.push(t);
	}

	const config: BackupConfig = {
		mode: p[1],
		mDiv: p[2],
		mFric: p[3],
		persistMode: p[22],
		chord: [p[23], p[24], p[25]],
		types,
	};
	if (p[0] >= 18 && p.length > 183) config.chordD = [p[180], p[181], p[182], p[183]];
	if (p[0] >= 19 && p.length > 184) config.swGyroLegacy = p[184];

	// Version 2 carries the lizard map. The caller passes null when the map has
	// not finished its lazy load; snapshotting an empty one would clear the
	// target's bindings on restore.
	if (lizardMap) {
		config.lizardMap = lizardMap.map((b) => ({
			outType: b.outType,
			od: b.od.slice(),
			trig: b.trig >>> 0,
			hold: b.hold >>> 0,
		}));
	}

	return { magic: BACKUP_MAGIC, version: config.lizardMap ? 2 : 1, bonds, config };
}

/** Validate and parse a backup file, throwing with a reason the UI can show. */
export function parseBackup(text: string): Backup {
	let obj: unknown;
	try {
		obj = JSON.parse(text);
	} catch {
		throw new Error('not valid JSON');
	}
	const b = obj as Backup;
	if (b?.magic !== BACKUP_MAGIC || !b.bonds || !b.config) throw new Error('unrecognized backup file');
	return b;
}

export const bondedCount = (b: Backup) => b.bonds.filter((x) => x.used).length;

/** Normalise a stored bond record to the fixed 24 bytes the write command wants. */
export function bondRecord(b: BackupBond | undefined): { used: number; rec: number[] } {
	const rec = hexDec(b?.rec ?? '');
	const r24 = new Array(24).fill(0);
	for (let i = 0; i < 24 && i < rec.length; i++) r24[i] = rec[i];
	// A short or missing record cannot be a valid bond, whatever the flag says.
	return { used: b?.used && rec.length === 24 ? 1 : 0, rec: r24 };
}

/** Total write steps, so the progress bar tracks real work rather than guessing. */
export function restoreSteps(c: BackupConfig, lizardCapable: boolean): number {
	const typeN = Array.isArray(c.types) ? Math.min(c.types.length, 4) : 0;
	const lizardN = Array.isArray(c.lizardMap) && lizardCapable ? Math.min(c.lizardMap.length, LZ_MAX) : 0;
	const baseN = 6 + (Array.isArray(c.chordD) ? 4 : 0) + (c.swGyroLegacy !== undefined ? 1 : 0);
	const padStickN = Array.isArray(c.types)
		? c.types.slice(0, typeN).filter((t) => Array.isArray(t.padStick)).length * 2
		: 0;
	// + 4 bond slots + the final commit
	return baseN + typeN * 9 + padStickN + lizardN + 4 + 1;
}

export const backupFilename = () =>
	`openpuck-backup-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.json`;

/** Backup/restore opcodes. */
export const BK_OP = {
	dumpBonds: 0x09,
	writeBond: 0x0d,
	commitReboot: 0x0e,
} as const;

/** Sentinel for "leave the mode alone" in the commit command. */
export const MODE_UNCHANGED = 0xff;
