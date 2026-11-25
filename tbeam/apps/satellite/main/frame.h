/* SPDX-License-Identifier: MIT
 *
 * RAW_SENSOR packer (protocols/specs/messages.md, ADR 0029) — the C-side
 * mirror of python-scripts/ground_station/ground_station/lora_frames.py's
 * encode_raw_sensor(). Same field order, same scale factors, same
 * sentinels; keep the two in lockstep by construction, not by review.
 */

#ifndef SWARM_FRAME_H_
#define SWARM_FRAME_H_

#include <stdbool.h>
#include <stdint.h>

#include "protocol/sensor_bundle.h"

#define SWARM_RAW_SENSOR_SIZE 26

/* The external BME688's freshest T/RH reading. Unlike pressure and gas —
 * which the BME688 task overlays directly into sensor_bundle_t.pressure_raw
 * / gas_raw (ADR 0014/0027/0029) — temp/humidity never touch the bundle:
 * sensor_bundle_t.temp_raw/humid_raw are reserved for the SCD30's own
 * secondary reading and are explicitly not transmitted (messages.md). This
 * is the separate path temp/humidity take to reach the frame. */
typedef struct {
    float temp_c;
    float humidity_pct;
} swarm_env_overlay_t;

/**
 * Pack one RAW_SENSOR frame (26 bytes, ADR 0029).
 *
 * @param node_id         low nibble of the header byte (1-9 for a
 *                        provisioned satellite; the caller is responsible
 *                        for never calling this at node_id 15)
 * @param timestamp       Unix epoch seconds (gps_time)
 * @param lat_1e7,lon_1e7 degrees x 1e7; pass 0 for both when gps_fix_valid
 *                        is false, matching the encoder's own convention
 * @param gps_fix_valid   sets flags bit 4
 * @param time_valid      sets flags bit 5
 * @param env             freshest BME688 temp/humidity (see struct comment)
 * @param bundle          the Core 1 hand-off: supplies pm2_5, co2, the
 *                         BME688-overlaid pressure_raw/gas_raw, seq, and
 *                         flags bits 0/1/2/3/7 (PMS7003/SCD30/ENV/THERMAL/GAS)
 * @param out             exactly SWARM_RAW_SENSOR_SIZE bytes
 */
void swarm_frame_pack_raw_sensor(uint8_t node_id, uint32_t timestamp,
                                  int32_t lat_1e7, int32_t lon_1e7,
                                  bool gps_fix_valid, bool time_valid,
                                  const swarm_env_overlay_t *env,
                                  const sensor_bundle_t *bundle,
                                  uint8_t out[SWARM_RAW_SENSOR_SIZE]);

#endif /* SWARM_FRAME_H_ */
