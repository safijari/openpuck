/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Braden Mund
 */

#ifndef JC2_ATT_H
#define JC2_ATT_H

#include "jc2_ll_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Process one reassembled L2CAP ATT PDU on CID 0x0004 for connection c. */
void jc2_att_handle_pdu(jc2_ll_conn_t *c, const uint8_t *att, uint16_t len,
			jc2_link_diag_t *diag, const jc2_ll_callbacks_t *cbs);

#ifdef __cplusplus
}
#endif

#endif /* JC2_ATT_H */
