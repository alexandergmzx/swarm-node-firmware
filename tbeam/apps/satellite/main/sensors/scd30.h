/* SPDX-License-Identifier: MIT
 *
 * SCD30 (Sensirion) CO2 + secondary T/RH, I2C 0x61 (ADR 0027). Hand-rolled
 * against the primary source (docs/app-notes/sensirion/
 * sensirion_scd30_interface-description_v1.0.pdf) rather than vendored:
 * Sensirion's own embedded-scd driver expects a Makefile "release" step to
 * flatten a nested submodule into one buildable tree, which fits
 * idf_component_register poorly for a protocol this small (five commands
 * actually used below).
 */

#ifndef SWARM_SENSOR_SCD30_H_
#define SWARM_SENSOR_SCD30_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Starts continuous measurement (2 s interval, matching ADR 0016's
 * cadence) and the polling task, Core 1, low priority. */
esp_err_t swarm_scd30_start(void);

/** Called by the BME688 task after each pressure reading, so the SCD30's
 * CO2 compensation stays fed without a CALIBRATION_EVENT round trip (ADR
 * 0014/0023/0027) — the two sensors share one I2C bus, no cross-MCU hop
 * needed. Re-sends the trigger-continuous-measurement command with the new
 * pressure argument, per the interface description's note that the whole
 * command must be resent to change it mid-stream. */
esp_err_t swarm_scd30_set_ambient_pressure(uint16_t pressure_mbar);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_SENSOR_SCD30_H_ */
