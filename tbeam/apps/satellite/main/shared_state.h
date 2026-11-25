/* SPDX-License-Identifier: MIT
 *
 * The Core 1 -> Core 0 hand-off (ADR 0023 item 4): the freshest sensor
 * bundle and BME688 overlay, written by the acquisition tasks, read once
 * per superframe by tdma_task when it packs a frame (ADR 0016 — "the
 * packer takes the freshest 2 s sensor_bundle_t").
 */

#ifndef SWARM_SHARED_STATE_H_
#define SWARM_SHARED_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include "frame.h"
#include "protocol/sensor_bundle.h"

#ifdef __cplusplus
extern "C" {
#endif

void swarm_state_init(void);

/* Acquisition tasks call these after each successful read. Each setter
 * read-modify-writes only its own sensor's fields plus its VALID bit and a
 * per-sensor last-updated timestamp, so PMS7003/SCD30/BME688 — three
 * independent tasks — never clobber each other. */
void swarm_state_set_pms7003(uint16_t pm25_raw);
void swarm_state_set_scd30(uint16_t co2_raw, int16_t temp_raw, uint16_t humid_raw);
void swarm_state_set_bme688(float temp_c, float humidity_pct,
                             uint16_t pressure_raw, uint16_t gas_raw,
                             bool gas_valid);

/**
 * tdma_task calls this once per superframe, immediately before packing a
 * frame. Applies the staleness rule (messages.md): a sensor whose setter
 * has not fired in > 6 s has stalled or stopped answering, so its VALID
 * bit is cleared here — validity bits are only ever cleared, never set, by
 * this function. Stamps and returns the next seq_number.
 */
void swarm_state_snapshot_for_tx(sensor_bundle_t *bundle_out,
                                  swarm_env_overlay_t *env_out);

/* Read-only copy for the status screen: the same staleness rule applied to
 * the copy's flags, nothing stored changes and seq_number is not advanced
 * (only the TX path may advance it). */
void swarm_state_peek(sensor_bundle_t *bundle_out, swarm_env_overlay_t *env_out);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_SHARED_STATE_H_ */
