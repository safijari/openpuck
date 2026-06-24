// nrf_esb_shim.h -- minimal nRF5-SDK shim so the vendored Nordic nrf_esb.c/.h build against the Adafruit
// nRF52 BSP WITHOUT the rest of the nRF5 SDK. nrf_esb references only a small surface from the SDK: the
// nrf_error.h codes, the sdk_macros VERIFY_* helpers, and STATIC_ASSERT (from app_util.h). Its debug pins
// are compiled out (NRF_ESB_DEBUG is undefined) so nrf_gpio.h / nrf_delay.h are not actually used. The
// vendored files include THIS instead of those SDK headers (the only edit to the vendored code, marked
// "OpenPuck shim" there). See docs/ESB_MIGRATION.md.
#pragma once
#include <stdint.h>
#include <stddef.h>

// Error codes: use the BSP/SoftDevice nrf_error.h (it is on the global include path) rather than redefining
// them -- esb_backend.cpp also pulls it in via Arduino.h, so defining our own here clashed.
// NRF_ERROR_BUFFER_EMPTY is supplied separately by nrf_esb_error_codes.h.
#include "nrf_error.h"

// sdk_errors.h typedef (nrf_esb.c uses ret_code_t once). Identical re-typedef is harmless in C11/C++.
typedef uint32_t ret_code_t;

// The controller's 0x43-augmented F1 reply is ~66 bytes; nrf_esb defaults the payload cap to 32 (which would
// truncate it). Define before nrf_esb.h picks its default. Matches the raw backend's 96-byte ceiling.
#ifndef NRF_ESB_MAX_PAYLOAD_LENGTH
#define NRF_ESB_MAX_PAYLOAD_LENGTH 96
#endif

// sdk_macros.h helpers: bail out of the calling function with an error on a failed precondition.
#define VERIFY_TRUE(statement, err_code)   \
	do {                               \
		if (!(statement))          \
			return (err_code); \
	} while (0)
#define VERIFY_FALSE(statement, err_code)  \
	do {                               \
		if ((statement))           \
			return (err_code); \
	} while (0)
#define VERIFY_PARAM_NOT_NULL(param)           \
	do {                                   \
		if ((param) == NULL)           \
			return NRF_ERROR_NULL; \
	} while (0)

#ifdef __cplusplus
#define STATIC_ASSERT(EXPR) static_assert((EXPR), #EXPR)
#else
#define STATIC_ASSERT(EXPR) _Static_assert((EXPR), #EXPR)
#endif
