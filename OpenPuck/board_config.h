#pragma once

#if defined(OPK_BOARD_MDBT50Q_CX_40) || defined(OPK_BOARD_PCA10059)
// The staged updater rewrites an Adafruit-format bootloader settings page.
#define OPK_HAS_ADAFRUIT_DFU 0
#else
#define OPK_HAS_ADAFRUIT_DFU 1
#endif

#if defined(OPK_BOARD_PCA10059)
// Nordic's Open DFU bootloader on the PCA10059 reserves 0xE0000 and up, so
// the fault black-box page sits just below the relocated InternalFS
// (0xD9000-0xDFFFF, see `make build-pca10059`).
#define OPK_BB_ADDR 0xD8000UL
#else
// app image (~170 KB, ends < 0x60000) < here < InternalFS (0xED000)
#define OPK_BB_ADDR 0xE8000UL
#endif
