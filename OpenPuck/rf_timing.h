#pragma once

#include <stdint.h>

// Marks a short controller feature transaction whose reply may need the
// hardware-bounded acquisition/completion extension used by rfConnTx().
void rfTimingBeginFeatureResponse();
void rfTimingEndFeatureResponse();

bool rfTimingArm();
void rfTimingCaptureDecision();
bool rfTimingBeginCompletionGuard(uint16_t maxAfterAddressUs,
				  uint32_t *deadlineTicks);
bool rfTimingCompletionDeadlineReached(uint32_t deadlineTicks);
bool rfTimingWaitForAcquisition(uint16_t afterReadyUs,
				uint16_t packetAfterAddressUs,
				uint16_t readyFailsafeFromBaseUs);
void rfTimingFinish();
