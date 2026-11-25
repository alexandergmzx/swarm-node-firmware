/* SPDX-License-Identifier: MIT
 *
 * The one I2C bus every classic satellite carries: PMU (0x34), SCD30
 * (0x61), BME688 (0x76) — ADR 0027. One bus master, added here once;
 * scd30.c and bme688.c each attach their own device handle to it.
 */

#ifndef SWARM_I2C_BUS_H_
#define SWARM_I2C_BUS_H_

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t swarm_i2c_bus_init(void);

/** NULL until swarm_i2c_bus_init() has succeeded. */
i2c_master_bus_handle_t swarm_i2c_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_I2C_BUS_H_ */
