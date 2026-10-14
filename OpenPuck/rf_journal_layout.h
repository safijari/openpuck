// Fixed flash layout for the RF channel-history journal.
//
// Normal application images are capped immediately below this window, while
// staged WebUSB update data starts above it. Keeping the journal outside both
// regions lets learned RF history survive ordinary firmware updates. A full
// board wipe still erases this application-flash window intentionally.
#pragma once
#include <stdint.h>

#define RF_CHANNEL_JOURNAL_PAGE_BYTES 4096u
#define RF_CHANNEL_JOURNAL_PAGE_COUNT 2u
#ifndef OPK_RF_JOURNAL_BASE
#define OPK_RF_JOURNAL_BASE 0x86000UL
#endif
#define RF_CHANNEL_JOURNAL_BASE OPK_RF_JOURNAL_BASE
#define RF_CHANNEL_JOURNAL_END     \
	(RF_CHANNEL_JOURNAL_BASE + \
	 RF_CHANNEL_JOURNAL_PAGE_BYTES * RF_CHANNEL_JOURNAL_PAGE_COUNT)

static_assert((RF_CHANNEL_JOURNAL_BASE &
	       (RF_CHANNEL_JOURNAL_PAGE_BYTES - 1u)) == 0u,
	      "RF journal base must be page aligned");
