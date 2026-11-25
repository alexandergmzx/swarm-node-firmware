/* SPDX-License-Identifier: MIT
 *
 * GPS time discipline (ADR 0016 / ADR 0027). Classic satellites carry no
 * PCF8563 and PPS routing is UNVERIFIED (bring-up item, M0), so the plan of
 * record is permanent NMEA-disciplined fallback: every valid $--RMC fix
 * re-anchors the epoch estimate, and the error bound grows with elapsed
 * time between fixes until the next one arrives. If a board's PPS pad is
 * found and wired (board_pins.h SWARM_PIN_GPS_PPS >= 0), the PPS edge
 * tightens the same estimate instead of replacing this module.
 *
 * Shared component (firmware/tbeam/components/gps_time). The satellite
 * reads it for time: tdma_task takes the epoch/error estimate every
 * superframe. The receiver (node_id 0) reads it for position only: it
 * prints its own fix on the USB tether as a FIX line every superframe
 * (protocols/specs/messages.md) so the ground station knows where the
 * receiver is (the epoch/error fields ride along as a clock-quality
 * readout) — node 0 still never transmits (ADR 0016) and has no slot to
 * discipline for.
 */

#ifndef SWARM_GPS_TIME_H_
#define SWARM_GPS_TIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A time reference anchored to a boot-relative instant, so callers can
 * extrapolate an exact future wall-clock instant to a precise
 * esp_timer_get_time() target without re-querying (and without the extra
 * latency of a fresh query compounding into the error estimate). */
typedef struct {
    int64_t boot_us;     /* esp_timer_get_time() at the reference instant */
    int64_t epoch_ms;    /* Unix epoch milliseconds at that instant */
    uint32_t error_ms;   /* estimated bound on epoch_ms's error, +/- */
    bool disciplined;    /* true = PPS-quality; false = NMEA fallback */
    bool time_valid;     /* false until the first fix (or forever, if none) */
    bool position_valid; /* latest fix carried a valid GNSS position */
    int32_t lat_1e7;     /* degrees x 1e7; 0 if !position_valid */
    int32_t lon_1e7;
} swarm_time_ref_t;

/** Starts the UART NMEA reader task (and the PPS ISR, if the board routes
 * one) on Core 0 at real-time priority — GPS discipline feeds the TDMA slot
 * timer, so it lives on the hard-real-time core (ADR 0028). */
esp_err_t swarm_gps_time_start(void);

/** Snapshot of the current best time/position estimate. Safe to call from
 * any task; internally guarded by a critical section against the UART/ISR
 * writers. */
esp_err_t swarm_gps_time_get_ref(swarm_time_ref_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_GPS_TIME_H_ */
