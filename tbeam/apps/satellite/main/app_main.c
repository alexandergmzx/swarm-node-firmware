/* SPDX-License-Identifier: MIT
 *
 * A TDMA satellite (ADR 0027/0028). Core 0: PPS ISR (if routed) + GPS time
 * discipline + TDMA slot timing + radio TX — hard real-time. Core 1: the
 * external sensor kit (PMS7003, SCD30, BME688) + frame packing — slow,
 * blocking I/O that must never touch Core 0 (tenet 2, CLAUDE.md).
 *
 * See firmware/tbeam/README.md for the full task/priority table and why
 * this is a separate app from the receiver rather than a role flag.
 */

#include "esp_log.h"
#include "gps_time.h"
#include "i2c_bus.h"
#include "lora_radio.h"
#include "pmu.h"
#include "sensors/bme688.h"
#include "sensors/pms7003.h"
#include "sensors/scd30.h"
#include "shared_state.h"
#include "tdma.h"

static const char *TAG = "satellite";

void app_main(void) {
    ESP_LOGI(TAG, "swarm satellite starting");

    swarm_state_init();

    // The AXP2101 feeds the radio (ALDO2) and the GNSS module (ALDO3) and
    // both rails are off until enabled — this must precede the radio probe
    // and the GPS UART (components/pmu; ADR 0027 second bring-up note).
    if (swarm_i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed");
    }
    swarm_pmu_info_t pmu = {0};
    if (swarm_pmu_init(swarm_i2c_bus(), &pmu) != ESP_OK) {
        ESP_LOGE(TAG, "PMU init failed — the LoRa and GNSS rails may be off; expect the radio probe to fail");
    }

    lora_chip_t chip = LORA_CHIP_UNKNOWN;
    if (lora_radio_init(&chip) != ESP_OK) {
        ESP_LOGE(TAG, "radio init failed — no LoRa this boot, TDMA will run RX-only forever");
    } else {
        ESP_LOGI(TAG, "radio identified: %s", chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262");
    }

    if (swarm_gps_time_start() != ESP_OK) {
        ESP_LOGE(TAG, "gps_time start failed");
    }

    if (swarm_pms7003_start() != ESP_OK) ESP_LOGE(TAG, "pms7003 start failed");
    if (swarm_scd30_start() != ESP_OK) ESP_LOGE(TAG, "scd30 start failed");
    if (swarm_bme688_start() != ESP_OK) ESP_LOGE(TAG, "bme688 start failed");

    if (swarm_tdma_start() != ESP_OK) {
        ESP_LOGE(TAG, "tdma start failed — nothing left for app_main to do");
    }

    // Everything of substance now runs in the tasks started above;
    // app_main's own task can simply exit (ESP-IDF handles that cleanly).
}
