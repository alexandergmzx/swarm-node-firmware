/* SPDX-License-Identifier: MIT
 *
 * The USB-tethered ground-station receiver, node_id 0 (ADR 0027). The
 * entire job: continuous-RX LoRa, print every frame as one tether line,
 * plus one line per TDMA superframe saying where the receiver is. No TDMA,
 * no sensor pipeline — node_id 0 never transmits (ADR 0016), so none of
 * that applies here. The GNSS UART is opened for position only: the
 * shared gps_time component runs its NMEA reader, and this app reads the
 * estimate for the FIX line, never for slot discipline. See
 * firmware/tbeam/README.md for the full receiver-vs-satellite rationale.
 *
 * Tether contract (protocols/specs/messages.md, a dev contract — changeable
 * without an ADR), two line types over USB CDC-ACM:
 *
 *   one line per received frame
 *     RX <rssi_dbm> <snr_db> <hex-payload>\n
 *
 *   one line every FIX_PERIOD_MS (5000 = one TDMA superframe)
 *     FIX <lat> <lon> <epoch_s> <fix> <disc> <err_ms>\n
 *       lat, lon   decimal degrees, %.7f; 0.0000000 0.0000000 before the first fix
 *       epoch_s    Unix epoch seconds (integer) of the snapshot; 0 before the first fix
 *       fix        1 when the position is valid, else 0
 *       disc       1 when the time reference is PPS-disciplined, else 0
 *                  (a classic spare receiver routes no PPS: always 0)
 *       err_ms     estimated clock error bound, ms; 0 before the first fix
 *     e.g.  FIX 40.4168123 -3.7038456 1789000005 1 0 52
 *           FIX 0.0000000 0.0000000 0 0 0 0
 *     Exactly seven whitespace-separated fields. Both line types are
 *     printed from this one task, so a FIX line can never interleave
 *     with an RX line.
 *
 * Any line not starting with "RX " (including every ESP_LOG line below)
 * is ignored by a parser that only reads RX lines — that rule is what
 * makes FIX safe to add without touching the old ground-station path.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_time.h"
#include "lora_radio.h"
#include "pmu.h"

static const char *TAG = "receiver";

/* protocols/specs/messages.md: RAW_SENSOR is 26 B post-ADR-0029; MESH_ACK is
 * 8 B. Nothing this phase is bigger than RAW_SENSOR, but size generously —
 * a stray/garbled frame the size validator downstream will reject one day
 * should not itself corrupt the tether buffer. */
#define RX_BUF_LEN 64

/* One FIX line per TDMA superframe (ADR 0016: 5000 ms). */
#define FIX_PERIOD_MS 5000

/* The RX poll used to block forever (timeout 0) — there was nothing else to
 * do between frames. It now wakes at least once a second so the FIX cadence
 * holds during radio silence; a timeout costs nothing (lora_radio_rx_poll
 * leaves the radio armed and returns ESP_ERR_TIMEOUT without touching it). */
#define RX_POLL_TIMEOUT_MS 1000

static void print_tether_line(const uint8_t *data, size_t len, float rssi, float snr) {
    /* Hex-encode inline rather than pulling in a formatting helper — this is
     * the one place the wire format meets a human-readable line, and it
     * needs to be fast and allocation-free on a task that never sleeps long. */
    static const char hex[] = "0123456789abcdef";
    char hexbuf[2 * RX_BUF_LEN + 1];
    size_t n = len < RX_BUF_LEN ? len : RX_BUF_LEN;
    for (size_t i = 0; i < n; i++) {
        hexbuf[2 * i] = hex[data[i] >> 4];
        hexbuf[2 * i + 1] = hex[data[i] & 0x0F];
    }
    hexbuf[2 * n] = '\0';
    printf("RX %d %.1f %s\n", (int)rssi, snr, hexbuf);
    fflush(stdout);
}

static void print_fix_line(void) {
    static bool s_first_fix_logged = false;

    swarm_time_ref_t ref;
    if (swarm_gps_time_get_ref(&ref) != ESP_OK) return;

    /* Before the first RMC fix get_ref() hands back the zeroed reference, so
     * every field below prints as 0 — the documented before-first-fix line.
     * The reference is anchored to boot_us; extrapolate it to the print
     * instant so epoch_s is "now", not the last NMEA sentence's second. */
    long long epoch_s = 0;
    if (ref.time_valid) {
        int64_t now_ms = ref.epoch_ms + (esp_timer_get_time() - ref.boot_us) / 1000;
        epoch_s = (long long)(now_ms / 1000);
    }
    double lat = (double)ref.lat_1e7 / 1e7;
    double lon = (double)ref.lon_1e7 / 1e7;

    if (ref.position_valid && !s_first_fix_logged) {
        s_first_fix_logged = true;
        ESP_LOGI(TAG, "first GNSS fix: %.7f %.7f, clock %s +/-%u ms", lat, lon,
                 ref.disciplined ? "PPS-disciplined" : "NMEA", (unsigned)ref.error_ms);
    }

    /* Plain printf, not ESP_LOG: this is a tether line, not debug output. */
    printf("FIX %.7f %.7f %lld %d %d %u\n", lat, lon, epoch_s,
           ref.position_valid ? 1 : 0, ref.disciplined ? 1 : 0, (unsigned)ref.error_ms);
    fflush(stdout);
}

void app_main(void) {
    ESP_LOGI(TAG, "swarm receiver (node_id 0) starting");

    // The radio and the GNSS module hang from AXP2101 rails that are off
    // until enabled — S3 Supreme: ALDO3 (radio) + ALDO4 (GNSS); classic
    // spare receiver: ALDO2 (LoRa) + ALDO3 (GPS). components/pmu/pmu.cpp
    // enables both of this target's rails at 3.3 V in swarm_pmu_init(), so
    // the GNSS UART opened by swarm_gps_time_start() below needs no rail
    // work of its own. On the S3 the PMU sits on its own I2C bus
    // (IO42/IO41), which components/pmu opens.
    swarm_pmu_info_t pmu = {0};
    if (swarm_pmu_init(NULL, &pmu) != ESP_OK) {
        ESP_LOGE(TAG, "PMU init failed — the radio and GNSS rails may be off; expect the radio probe to fail");
    }

    lora_chip_t chip = LORA_CHIP_UNKNOWN;
    esp_err_t err = lora_radio_init(&chip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radio init failed (%s) — nothing to do, halting", esp_err_to_name(err));
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "radio identified: %s", chip == LORA_CHIP_SX1262 ? "SX1262" : "SX1278");

    // Position only (see the header): the NMEA reader task from
    // components/gps_time feeds the FIX line. Started after the radio, the
    // same order as the satellite, so the GPIO ISR service the PPS edge
    // hooks into (S3 only) is already installed by the radio HAL.
    if (swarm_gps_time_start() != ESP_OK) {
        ESP_LOGE(TAG, "gps_time start failed — FIX lines will report no fix");
    }

    err = lora_radio_rx_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rx_start failed (%s)", esp_err_to_name(err));
    }

    uint8_t buf[RX_BUF_LEN];
    // Primed so the first FIX line (the before-first-fix form — a receiver
    // heartbeat for the bridge) goes out on the first pass, then every
    // FIX_PERIOD_MS from there.
    int64_t last_fix_us = esp_timer_get_time() - (int64_t)FIX_PERIOD_MS * 1000;
    while (1) {
        size_t len = 0;
        float rssi = 0.0f, snr = 0.0f;
        esp_err_t rx_err = lora_radio_rx_poll(buf, sizeof(buf), &len, &rssi, &snr,
                                              RX_POLL_TIMEOUT_MS);
        if (rx_err == ESP_OK) {
            print_tether_line(buf, len, rssi, snr);
        } else if (rx_err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "rx_poll error: %s", esp_err_to_name(rx_err));
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_fix_us >= (int64_t)FIX_PERIOD_MS * 1000) {
            last_fix_us = now_us;
            print_fix_line();
        }
    }
}
