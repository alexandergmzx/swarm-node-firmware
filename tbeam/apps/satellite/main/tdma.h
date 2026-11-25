/* SPDX-License-Identifier: MIT
 *
 * TDMA slot state machine (ADR 0016). 5 s superframe, 10 slots x 500 ms,
 * slot = node_id. On GPS-disciplined time, TX starts at slot_start + 10 ms;
 * in NMEA fallback (the plan of record for classic satellites this phase —
 * ADR 0027), TX is delayed to slot_start + 210 ms and only sent while the
 * estimated clock error stays <= 200 ms, else the superframe is skipped
 * (RX-only). Sole owner of "may TX now" (ADR 0028) — the highest-priority
 * task on Core 0 besides the PPS ISR itself.
 */

#ifndef SWARM_TDMA_H_
#define SWARM_TDMA_H_

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Starts the TDMA task, pinned to Core 0. If CONFIG_SWARM_NODE_ID is 15
 * (unprovisioned), the task runs but never transmits — enforcing
 * protocols/specs/messages.md's node_id allocation table in code, not just
 * in the ADR. */
esp_err_t swarm_tdma_start(void);

/* Counters for the status screen, read under the same lock the TDMA task
 * writes them with. */
typedef struct {
    uint32_t tx_ok;        /* frames handed to the radio successfully */
    uint32_t tx_fail;      /* lora_radio_tx() errors */
    uint32_t skipped;      /* superframes skipped: clock error over budget */
    int64_t last_tx_us;    /* esp_timer time of the last good TX, -1 = none */
} swarm_tdma_stats_t;

void swarm_tdma_get_stats(swarm_tdma_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_TDMA_H_ */
