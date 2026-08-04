/* Linker script for the MakerDiary nRF52840 Connect Kit under OpenPuck (nosd).
 *
 * The Connect Kit's stock UF2 bootloader ships NO SoftDevice and hands control
 * to the application at 0x1000 (right after the MBR). Feather/XIAO boards place
 * an S140 in front of the app (0x26000 / 0x27000); this board has none, so the
 * app is linked at 0x1000 and reclaims that space. OpenPuck never uses a
 * SoftDevice, so this is the honest layout.
 *
 * FLASH region = MakerDiary's official "Application" partition
 * (nrf52840_partition_uf2_nosd.dtsi): 0x1000 .. 0xD4000 (844 kB). The 0xD4000 ..
 * 0xF4000 "Storage" partition (128 kB) is left free for the filesystem, and the
 * UF2 bootloader owns 0xF4000 .. 0x100000.
 *
 * RAM is left at the SoftDevice-era 0x20006000 origin for this first bring-up
 * (24 kB below it simply goes unused with no SD present) -- safe and OpenPuck
 * fits comfortably; can be reclaimed toward 0x20000000 later. INCLUDE
 * "nrf52_common.ld" resolves through the Adafruit core's linker dir
 * (-L{build.core.path}/linker).
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x1000, LENGTH = 0xD4000 - 0x1000

  RAM (rwx) :  ORIGIN = 0x20006000, LENGTH = 0x20040000 - 0x20006000
}

SECTIONS
{
  . = ALIGN(4);
  .svc_data :
  {
    PROVIDE(__start_svc_data = .);
    KEEP(*(.svc_data))
    PROVIDE(__stop_svc_data = .);
  } > RAM

  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

INCLUDE "nrf52_common.ld"
