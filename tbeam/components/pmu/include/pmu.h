/* SPDX-License-Identifier: MIT
 *
 * AXP2101 bring-up (both T-Beam variants carry one — ADR 0027). The radio
 * and the GNSS module are powered from LDOs under firmware control: ALDO2
 * (LoRa) + ALDO3 (GPS) on the classic T-Beam, ALDO3 (radio) + ALDO4 (GNSS)
 * on the S3 Supreme — vendor schematics, see board_pins.h. Measured cold on
 * classic-A (2026-09-03): the radio rail is OFF at power-up with a 2.8 V
 * setting, the GNSS rail already ON at 3.3 V — enabling both at 3.3 V is
 * what makes the radio exist and is harmless for the GNSS. swarm_pmu_init() must therefore run before lora_radio_init()
 * and before the GPS UART is opened.
 */

#ifndef SWARM_PMU_H_
#define SWARM_PMU_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  chip_id;          /**< PMU register 0x03: 0x4A = AXP2101 */
    bool     battery_present;
    bool     vbus_present;
    bool     charging;
    uint16_t batt_mv;
    uint16_t vbus_mv;
    uint16_t sys_mv;
} swarm_pmu_info_t;

/**
 * Bring the PMU up, enable this board's radio and GNSS rails at 3.3 V, and
 * turn on the voltage measurements.
 *
 * @param shared_bus  the sensor bus handle on boards where the PMU shares it
 *                    (classic T-Beam: pass swarm_i2c_bus()). May be NULL, in
 *                    which case — and always on boards with a dedicated PMU
 *                    bus (S3 Supreme) — this function opens a master bus on
 *                    SWARM_PIN_PMU_SDA/SCL itself.
 * @param out_info    optional; filled on success.
 * @return ESP_OK, ESP_ERR_NOT_FOUND when nothing answers at 0x34 as an
 *         AXP2101, or the I2C driver's error.
 */
esp_err_t swarm_pmu_init(i2c_master_bus_handle_t shared_bus, swarm_pmu_info_t *out_info);

/** The bus the PMU is on once swarm_pmu_init() succeeded; NULL before. */
i2c_master_bus_handle_t swarm_pmu_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_PMU_H_ */
