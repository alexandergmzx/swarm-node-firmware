/* SPDX-License-Identifier: MIT
 *
 * A TDMA satellite (ADR 0027/0028). Core 0: PPS ISR (if routed) + GPS time
 * discipline + TDMA slot timing + radio TX — hard real-time. Core 1: the
 * external sensor kit (PMS7003, SCD30, BME688) + frame packing — slow,
 * blocking I/O that must never touch Core 0 (architecture tenet 2).
 *
 * See tbeam/README.md for the full task/priority table and why
 * this is a separate app from the receiver rather than a role flag.
 */

#include "display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "frame.h"
#include "sdkconfig.h"
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

/* Bench status screen (CONFIG_SWARM_DISPLAY), redrawn once a second on Core 1:
 *   SAT node 4    915-MX     role, node id, band profile (title bar)
 *   SX1262 916.700 +2dBm     radio, frequency, TX power
 *   GPS NMEA +-37ms          time source and error bound, or "waiting"
 *   25.68661 -100.31611      position, or "no position"
 *   TX 42 3s air 0.57%       frames sent, age of the last, airtime share
 *                            (frames x computed time on air / time since the
 *                            first TX; "sk N" replaces the age when slots
 *                            were skipped)
 *   PM2.5 12.3  CO2 612      Core 1 sensor values ("--" when stale)
 *   T 24.1C  RH 45%          BME688
 *   vbus 5.08V  bat --       AXP2101 */
static lora_chip_t s_chip = LORA_CHIP_UNKNOWN;

static void render_status(void) {
    const lora_profile_t *prof = lora_radio_profile();
    swarm_display_row(0, "SAT node %d  %s", CONFIG_SWARM_NODE_ID, prof->name);
    const char *radio = s_chip == LORA_CHIP_SX1278 ? "SX1278" : s_chip == LORA_CHIP_SX1262 ? "SX1262" : "NO RADIO";
    swarm_display_row(1, "%s %lu.%03lu %+ddBm", radio, (unsigned long)(prof->freq_khz / 1000),
                      (unsigned long)(prof->freq_khz % 1000), prof->tx_dbm);

    swarm_time_ref_t ref;
    swarm_gps_time_get_ref(&ref);
    if (!ref.time_valid) {
        swarm_display_row(2, "GPS waiting for fix");
        swarm_display_row(3, "(silent until a fix)");
    } else {
        swarm_display_row(2, "GPS %s +-%lums", ref.disciplined ? "PPS" : "NMEA", (unsigned long)ref.error_ms);
        if (ref.position_valid) {
            swarm_display_row(3, "%.5f %.5f", ref.lat_1e7 / 1e7, ref.lon_1e7 / 1e7);
        } else {
            swarm_display_row(3, "no position");
        }
    }

    swarm_tdma_stats_t st;
    swarm_tdma_get_stats(&st);
    if (CONFIG_SWARM_NODE_ID == 15) {
        swarm_display_row(4, "UNPROVISIONED: no TX");
    } else if (st.last_tx_us < 0) {
        swarm_display_row(4, "TX none yet  sk %lu", (unsigned long)st.skipped);
    } else {
        int64_t now = esp_timer_get_time();
        long age_s = (long)((now - st.last_tx_us) / 1000000);
        /* Airtime share: one superframe's worth of wall time per frame at the
         * start, so the first frame does not read as 100 %. */
        int64_t span_us = (now - st.first_tx_us) + 5000000;
        double air_pct = 100.0 * (double)st.tx_ok * LORA_RADIO_AIRTIME_US(26) / (double)span_us;
        if (st.skipped > 0) {
            swarm_display_row(4, "TX %lu sk %lu air%.2f%%", (unsigned long)st.tx_ok, (unsigned long)st.skipped, air_pct);
        } else {
            swarm_display_row(4, "TX %lu %lds air %.2f%%", (unsigned long)st.tx_ok, age_s, air_pct);
        }
    }

    sensor_bundle_t b;
    swarm_env_overlay_t env;
    swarm_state_peek(&b, &env);
    if (!(b.flags & (SWARM_FLAG_PMS7003_VALID | SWARM_FLAG_SCD30_VALID | SWARM_FLAG_ENV_VALID))) {
        /* Nothing attached (or nothing fresh): say so rather than show a row
         * of dashes that reads like a fault. Frames still go out, with every
         * sensor VALID flag clear. */
        swarm_display_row(5, "no sensor kit");
        swarm_display_row(6, "frames: sensors clear");
    } else {
        char pm[8] = "--", co2[8] = "--";
        if (b.flags & SWARM_FLAG_PMS7003_VALID) snprintf(pm, sizeof(pm), "%u.%u", b.pm25_raw / 10, b.pm25_raw % 10);
        if (b.flags & SWARM_FLAG_SCD30_VALID) snprintf(co2, sizeof(co2), "%u", b.co2_raw);
        swarm_display_row(5, "PM2.5 %s  CO2 %s", pm, co2);
        if (b.flags & SWARM_FLAG_ENV_VALID) {
            swarm_display_row(6, "T %.1fC  RH %.0f%%", env.temp_c, env.humidity_pct);
        } else {
            swarm_display_row(6, "T/RH --  (no BME688)");
        }
    }

    swarm_pmu_info_t pmu;
    if (swarm_pmu_read(&pmu) == ESP_OK) {
        if (pmu.battery_present) {
            swarm_display_row(7, "vbus %.2fV bat %.2fV", pmu.vbus_mv / 1000.0, pmu.batt_mv / 1000.0);
        } else {
            swarm_display_row(7, "vbus %.2fV  bat --", pmu.vbus_mv / 1000.0);
        }
    }
}

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

    // The panel sits on the same sensor bus (classic: shared with the PMU;
    // S3: bus 0, powered by ALDO1 which swarm_pmu_init() just enabled).
    // A first frame right away, so the screen shows life before the radio
    // and GNSS come up.
    if (swarm_display_init(swarm_i2c_bus()) == ESP_OK) {
        swarm_display_row(0, "SAT node %d  booting", CONFIG_SWARM_NODE_ID);
        swarm_display_row(2, "radio + GNSS ...");
        swarm_display_flush();
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

    s_chip = chip;
    if (swarm_display_present() && swarm_display_start(render_status, 1000) != ESP_OK) {
        ESP_LOGW(TAG, "display task not started");
    }

    // Everything of substance now runs in the tasks started above;
    // app_main's own task can simply exit (ESP-IDF handles that cleanly).
}
