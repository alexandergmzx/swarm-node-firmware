/* SPDX-License-Identifier: MIT
 *
 * PMS7003 (Plantower) PM2.5, UART, `SET`-gated (ADR 0027). Hand-rolled: the
 * frame protocol is a fixed 32-byte struct with a checksum, simple enough
 * that vendoring a driver buys nothing over reading the datasheet once
 * (docs/datasheets/plantower/pms7003/).
 */

#ifndef SWARM_SENSOR_PMS7003_H_
#define SWARM_SENSOR_PMS7003_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Starts the UART reader task on Core 1, low priority (ADR 0028 — slow,
 * blocking sensor I/O never runs on Core 0). Drives SET high immediately
 * and reports readings as invalid until the ~30 s warm-up has elapsed. */
esp_err_t swarm_pms7003_start(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_SENSOR_PMS7003_H_ */
