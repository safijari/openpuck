/* Linker script for the Seeed XIAO nRF52840 under OpenPuck.
 *
 * Identical to the Adafruit core's nrf52840_s140_v6.ld except the FLASH ORIGIN:
 * the XIAO's UF2 bootloader ships SoftDevice S140 v7.3.0 and hands off to the
 * application at 0x27000 (Adafruit's S140 v6.1.1 boards use 0x26000). OpenPuck
 * never starts the SoftDevice, but the bootloader jumps to this fixed address,
 * so the app must be linked here. RAM is unchanged. INCLUDE "nrf52_common.ld"
 * resolves through the Adafruit core's linker dir (-L{build.core.path}/linker).
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx)     : ORIGIN = 0x27000, LENGTH = 0xED000 - 0x27000

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
