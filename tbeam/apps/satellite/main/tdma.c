// SPDX-License-Identifier: MIT

#include "tdma.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "frame.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_time.h"
#include "lora_radio.h"
#include "sdkconfig.h"
#include "shared_state.h"

static const char *TAG = "tdma";

#define SUPERFRAME_MS 5000
#define SLOT_MS 500
#define DISCIPLINED_START_OFFSET_MS 10
#define FALLBACK_START_OFFSET_MS 210
#define FALLBACK_MAX_ERROR_MS 200
#define UNPROVISIONED_NODE_ID 15

// The clock-fallback transmission (start offset + worst clock error + time on
// air of one RAW_SENSOR frame) must end inside its slot at the build's radio
// profile (ADR 0016): 210 + 200 + 61.7 ms at 433-LAB, 210 + 200 + 51.5 ms at
// 915-MX. SF9 at 125 kHz (205.8 ms) would fail here, not on the air.
_Static_assert(FALLBACK_START_OFFSET_MS + FALLBACK_MAX_ERROR_MS +
                   (LORA_RADIO_AIRTIME_US(SWARM_RAW_SENSOR_SIZE) + 999u) / 1000u <= SLOT_MS,
               "clock-fallback TX overruns the TDMA slot at this radio profile (ADR 0016)");

static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static swarm_tdma_stats_t s_stats = {.last_tx_us = -1};

void swarm_tdma_get_stats(swarm_tdma_stats_t *out) {
    portENTER_CRITICAL(&s_stats_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_stats_lock);
}

static void print_tx_echo(const uint8_t *data, size_t len) {
    static const char hex[] = "0123456789abcdef";
    char hexbuf[2 * SWARM_RAW_SENSOR_SIZE + 1];
    for (size_t i = 0; i < len; i++) {
        hexbuf[2 * i] = hex[data[i] >> 4];
        hexbuf[2 * i + 1] = hex[data[i] & 0x0F];
    }
    hexbuf[2 * len] = '\0';
    // Plain printf, not ESP_LOG: the Milestone-A DoD compares this line
    // byte-exact against what the ground station decoded off the air.
    printf("TX %s\n", hexbuf);
    fflush(stdout);
}

// Sleeps until target_boot_us, in one shot if it is already close, or in a
// coarse-then-fine two-step if it is more than a tick away — avoids waking
// every FreeRTOS tick for a target that may be seconds out.
static void delay_until(int64_t target_boot_us) {
    int64_t now = esp_timer_get_time();
    if (target_boot_us <= now) return;
    int64_t remaining_ms = (target_boot_us - now) / 1000;
    if (remaining_ms > 2) {
        vTaskDelay(pdMS_TO_TICKS(remaining_ms - 2));
    }
    while (esp_timer_get_time() < target_boot_us) {
        // sub-tick busy-wait for the last ~2 ms; short enough not to starve
        // anything at this task's priority.
    }
}

static void tdma_task(void *arg) {
    (void)arg;
    uint8_t node_id = (uint8_t)CONFIG_SWARM_NODE_ID;
    bool provisioned = node_id != UNPROVISIONED_NODE_ID && node_id >= 1 && node_id <= 9;
    if (!provisioned) {
        ESP_LOGW(TAG, "node_id %u is not a provisioned satellite slot (1-9) — "
                       "running RX-only, will never transmit (messages.md)", node_id);
    }

    int64_t last_superframe_start_ms = -1;

    while (1) {
        swarm_time_ref_t ref;
        swarm_gps_time_get_ref(&ref);

        if (!ref.time_valid) {
            // Visible at INFO and rate-limited: a satellite without a fix is
            // otherwise silent, which the desk session of 2026-09-03 showed is
            // indistinguishable from a dead one (bring-up log).
            static int64_t s_last_wait_log_us = -60LL * 1000 * 1000;
            int64_t now_us = esp_timer_get_time();
            if (now_us - s_last_wait_log_us >= 30LL * 1000 * 1000) {
                ESP_LOGI(TAG, "no GPS time yet — waiting for an RMC fix, not transmitting (node_id %u)",
                         node_id);
                s_last_wait_log_us = now_us;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int64_t superframe_start_ms = (ref.epoch_ms / SUPERFRAME_MS) * SUPERFRAME_MS;
        if (superframe_start_ms <= last_superframe_start_ms) {
            superframe_start_ms += SUPERFRAME_MS;  // don't re-transmit within one superframe
        }
        int64_t slot_start_ms = superframe_start_ms + (int64_t)node_id * SLOT_MS;
        int64_t start_offset_ms = ref.disciplined ? DISCIPLINED_START_OFFSET_MS
                                                   : FALLBACK_START_OFFSET_MS;
        int64_t tx_target_ms = slot_start_ms + start_offset_ms;

        int64_t target_boot_us = ref.boot_us + (tx_target_ms - ref.epoch_ms) * 1000;
        delay_until(target_boot_us);
        last_superframe_start_ms = superframe_start_ms;

        if (!provisioned) {
            continue;  // RX-only per messages.md's node_id table
        }

        // Re-check error at the actual TX instant, not when we started
        // waiting — a multi-second sleep is exactly where drift matters
        // (ADR 0016's clock-fallback budget).
        swarm_gps_time_get_ref(&ref);
        if (!ref.disciplined && ref.error_ms > FALLBACK_MAX_ERROR_MS) {
            ESP_LOGW(TAG, "clock error %lu ms exceeds fallback budget — skipping this superframe (RX-only)",
                     (unsigned long)ref.error_ms);
            portENTER_CRITICAL(&s_stats_lock);
            s_stats.skipped++;
            portEXIT_CRITICAL(&s_stats_lock);
            continue;
        }

        sensor_bundle_t bundle;
        swarm_env_overlay_t env;
        swarm_state_snapshot_for_tx(&bundle, &env);

        uint8_t frame[SWARM_RAW_SENSOR_SIZE];
        swarm_frame_pack_raw_sensor(node_id, (uint32_t)(tx_target_ms / 1000),
                                     ref.lat_1e7, ref.lon_1e7, ref.position_valid,
                                     ref.time_valid, &env, &bundle, frame);

        esp_err_t err = lora_radio_tx(frame, sizeof(frame));
        portENTER_CRITICAL(&s_stats_lock);
        if (err == ESP_OK) {
            s_stats.tx_ok++;
            s_stats.last_tx_us = esp_timer_get_time();
        } else {
            s_stats.tx_fail++;
        }
        portEXIT_CRITICAL(&s_stats_lock);
        if (err == ESP_OK) {
            print_tx_echo(frame, sizeof(frame));
        } else {
            ESP_LOGE(TAG, "transmit failed: %s", esp_err_to_name(err));
        }
    }
}

esp_err_t swarm_tdma_start(void) {
    // Core 0, the highest application priority besides the PPS ISR itself —
    // sole owner of "may TX now" (ADR 0028).
    BaseType_t ok = xTaskCreatePinnedToCore(tdma_task, "tdma", 4096, NULL, 23, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
