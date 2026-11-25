// SPDX-License-Identifier: MIT

#include "shared_state.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define STALE_US (6 * 1000 * 1000)  // messages.md: no update in > 6 s -> stale

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Guarded by s_lock. bundle.seq_number is bumped once per snapshot, not per
// sensor write — one RAW_SENSOR per superframe (ADR 0016), so the sequence
// tracks transmitted frames, not individual readings.
static sensor_bundle_t s_bundle;
static swarm_env_overlay_t s_env;
static int64_t s_pms_updated_us = -1;
static int64_t s_scd30_updated_us = -1;

void swarm_state_init(void) {
    portENTER_CRITICAL(&s_lock);
    s_bundle = (sensor_bundle_t){0};
    s_env = (swarm_env_overlay_t){0};
    s_pms_updated_us = -1;
    s_scd30_updated_us = -1;
    portEXIT_CRITICAL(&s_lock);
}

void swarm_state_set_pms7003(uint16_t pm25_raw) {
    portENTER_CRITICAL(&s_lock);
    s_bundle.pm25_raw = pm25_raw;
    s_bundle.flags |= SWARM_FLAG_PMS7003_VALID;
    s_pms_updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

void swarm_state_set_scd30(uint16_t co2_raw, int16_t temp_raw, uint16_t humid_raw) {
    portENTER_CRITICAL(&s_lock);
    s_bundle.co2_raw = co2_raw;
    s_bundle.temp_raw = temp_raw;    // SCD30's own secondary reading — not transmitted
    s_bundle.humid_raw = humid_raw;  // (messages.md); kept for on-node compensation use only
    s_bundle.flags |= SWARM_FLAG_SCD30_VALID;
    s_scd30_updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

void swarm_state_set_bme688(float temp_c, float humidity_pct,
                             uint16_t pressure_raw, uint16_t gas_raw,
                             bool gas_valid) {
    portENTER_CRITICAL(&s_lock);
    s_env.temp_c = temp_c;
    s_env.humidity_pct = humidity_pct;
    s_bundle.pressure_raw = pressure_raw;  // overlay, per ADR 0014/0027 (0 = invalid)
    s_bundle.gas_raw = gas_raw;            // overlay, per ADR 0029 (0 = invalid)
    s_bundle.flags |= SWARM_FLAG_ENV_VALID;
    if (gas_valid) {
        s_bundle.flags |= SWARM_FLAG_GAS_VALID;
    } else {
        // Heater warm-up: T/RH/P are good, gas is not yet settled — only
        // ever clear this bit here, never set it, on a not-yet-valid read.
        s_bundle.flags &= (uint8_t)~SWARM_FLAG_GAS_VALID;
    }
    portEXIT_CRITICAL(&s_lock);
}

void swarm_state_peek(sensor_bundle_t *bundle_out, swarm_env_overlay_t *env_out) {
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    *bundle_out = s_bundle;
    *env_out = s_env;
    bool pms_stale = s_pms_updated_us < 0 || (now - s_pms_updated_us) > STALE_US;
    bool scd_stale = s_scd30_updated_us < 0 || (now - s_scd30_updated_us) > STALE_US;
    portEXIT_CRITICAL(&s_lock);
    if (pms_stale) bundle_out->flags &= (uint8_t)~SWARM_FLAG_PMS7003_VALID;
    if (scd_stale) bundle_out->flags &= (uint8_t)~SWARM_FLAG_SCD30_VALID;
}

void swarm_state_snapshot_for_tx(sensor_bundle_t *bundle_out,
                                  swarm_env_overlay_t *env_out) {
    int64_t now = esp_timer_get_time();

    portENTER_CRITICAL(&s_lock);
    if (s_pms_updated_us < 0 || (now - s_pms_updated_us) > STALE_US) {
        s_bundle.flags &= (uint8_t)~SWARM_FLAG_PMS7003_VALID;
    }
    if (s_scd30_updated_us < 0 || (now - s_scd30_updated_us) > STALE_US) {
        s_bundle.flags &= (uint8_t)~SWARM_FLAG_SCD30_VALID;
    }
    s_bundle.seq_number++;  // wraps at 255, matches the u8 field (sensor_bundle.h)
    *bundle_out = s_bundle;
    *env_out = s_env;
    portEXIT_CRITICAL(&s_lock);
}
