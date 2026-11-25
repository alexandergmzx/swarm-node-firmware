/* SPDX-License-Identifier: MIT
 *
 * BME688 (Bosch) — the authoritative T/RH/P source and the gas-resistance
 * channel (ADR 0027/0029), I2C 0x76 (SDO -> GND; the S3's onboard BME280
 * owns 0x77 on the same class of bus). Uses the vendored Bosch
 * BME68x_SensorAPI (third_party/, pinned v4.4.8) in forced mode with the
 * gas heater on.
 */

#ifndef SWARM_SENSOR_BME688_H_
#define SWARM_SENSOR_BME688_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Starts the forced-mode measurement task, Core 1, low priority. Feeds
 * shared_state's T/RH/P/gas overlay every cycle and calls
 * swarm_scd30_set_ambient_pressure() so the SCD30 stays compensated
 * without a CALIBRATION_EVENT round trip (ADR 0014/0023/0027). */
esp_err_t swarm_bme688_start(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_SENSOR_BME688_H_ */
