// SPDX-License-Identifier: MIT
//
// NMEA $--RMC parsing + an optional PPS tightening path. See gps_time.h for
// the contract and why NMEA fallback is the plan of record this phase.

#include "gps_time.h"

#include <math.h>
#include <string.h>
#include <time.h>  // struct tm only — see timegm_utc() below for why not timegm()

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gps_time";

#define GPS_UART_NUM UART_NUM_1
#define GPS_UART_BAUD 9600
#define NMEA_LINE_MAX 96

// Calibration constants — placeholders pending the M0 bench pass (ADR 0027
// decision 6): NMEA_FIX_LATENCY_MS is the typical gap between the UTC
// instant an RMC sentence reports and the moment this task finishes parsing
// it; DRIFT_MS_PER_MIN is a conservative stand-in for the classic T-Beam's
// uncalibrated crystal (the plan's own math: ~10 ppm ~= 0.6 ms/min, comfortably
// inside ADR 0016's +/-200 ms budget even several times over — this constant
// is deliberately pessimistic until a bench measurement replaces it).
#define NMEA_FIX_LATENCY_MS 50
#define DRIFT_MS_PER_MIN 2
// Cap on the *reported* estimate — a bound on the arithmetic, not the transmit
// budget. The budget is tdma.c's FALLBACK_MAX_ERROR_MS (200, ADR 0016); clamping
// here at 200 made that check unreachable, so a satellite kept transmitting
// however stale its fix was (found in the 2026-09-03 firmware survey).
#define MAX_ERROR_MS 60000
#define PPS_FRESH_WINDOW_MS 1100

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static swarm_time_ref_t s_ref = {0};        // guarded by s_lock
static volatile int64_t s_pps_boot_us = -1; // guarded by s_lock; -1 = none yet

// Days since 1970-01-01 for a proleptic-Gregorian civil date (Howard
// Hinnant's public-domain days_from_civil algorithm — no timezone/DST, UTC
// only, exactly what NMEA dates are). Used instead of libc's timegm(),
// which this toolchain's picolibc does not expose even under -D_GNU_SOURCE
// (its declaration is gated on __GNU_VISIBLE, which is not set here for
// reasons not worth chasing when a ten-line, independently-verified
// replacement is this portable). Verified this session against Python's
// calendar.timegm across five dates spanning 1994-2038, including the
// year-2038 rollover.
static int64_t days_from_civil(int y, int m, int d) {
    y -= (m <= 2);
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    int yoe = (int)(y - era * 400);
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

static int64_t timegm_utc(const struct tm *tm) {
    int64_t days = days_from_civil(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);
    return days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

static void IRAM_ATTR pps_isr(void *arg) {
    (void)arg;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_lock);
    s_pps_boot_us = now;
    portEXIT_CRITICAL_ISR(&s_lock);
}

// -- NMEA field parsing ------------------------------------------------

// Splits `line` in place on commas; fields[i] points into line. Returns the
// field count. Deliberately not strtok: this runs on one task only, but a
// hand-rolled scan keeps the '*checksum' suffix out of the last field
// without a second pass.
static int split_fields(char *line, char *fields[], int max_fields) {
    int n = 0;
    char *p = line;
    fields[n++] = p;
    while (*p && n < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        } else if (*p == '*') {
            *p = '\0';
            break;
        }
        p++;
    }
    return n;
}

// ddmm.mmmm or dddmm.mmmm -> degrees x 1e7. `int_digits` is 2 for latitude,
// 3 for longitude (the field widths NMEA fixes them at).
static int32_t parse_coord(const char *field, char hemisphere, int int_digits) {
    if (field[0] == '\0') return 0;
    double raw = atof(field);
    double degrees_part = floor(raw / 100.0);
    // atof already gave us the full ddmm.mmmm; recompute the integer part
    // from the string width instead, so a longitude like "01131.0000"
    // parses as 011 deg 31.0000 min, not 11 deg 31.0000 min.
    char int_buf[4] = {0};
    strncpy(int_buf, field, int_digits);
    degrees_part = atof(int_buf);
    double minutes_part = raw - (degrees_part * 100.0);
    double decimal_deg = degrees_part + minutes_part / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') decimal_deg = -decimal_deg;
    return (int32_t)lround(decimal_deg * 1e7);
}

static bool parse_rmc(char *line) {
    char *fields[16];
    int n = split_fields(line, fields, 16);
    if (n < 10) return false;
    if (fields[2][0] != 'A') return false;  // 'V' = no fix; nothing to do

    const char *time_s = fields[1];  // hhmmss.ss
    const char *date_s = fields[9];  // ddmmyy
    if (strlen(time_s) < 6 || strlen(date_s) < 6) return false;

    char buf[3] = {0};
    struct tm tm = {0};
    strncpy(buf, time_s, 2); tm.tm_hour = atoi(buf);
    strncpy(buf, time_s + 2, 2); tm.tm_min = atoi(buf);
    strncpy(buf, time_s + 4, 2); tm.tm_sec = atoi(buf);
    int frac_ms = 0;
    const char *dot = strchr(time_s, '.');
    if (dot != NULL) frac_ms = (int)lround(atof(dot) * 1000.0);

    strncpy(buf, date_s, 2); tm.tm_mday = atoi(buf);
    strncpy(buf, date_s + 2, 2); tm.tm_mon = atoi(buf) - 1;
    strncpy(buf, date_s + 4, 2); tm.tm_year = atoi(buf) + 100;  // 2000-2099

    int64_t epoch_s = timegm_utc(&tm);
    if (epoch_s <= 0) return false;

    int32_t lat_1e7 = 0, lon_1e7 = 0;
    bool has_pos = false;
    if (n > 6 && fields[3][0] != '\0' && fields[5][0] != '\0') {
        lat_1e7 = parse_coord(fields[3], fields[4][0], 2);
        lon_1e7 = parse_coord(fields[5], fields[6][0], 3);
        has_pos = true;
    }

    int64_t parse_done_us = esp_timer_get_time();
    int64_t epoch_ms = (int64_t)epoch_s * 1000 + frac_ms + NMEA_FIX_LATENCY_MS;

    portENTER_CRITICAL(&s_lock);
    s_ref.boot_us = parse_done_us;
    s_ref.epoch_ms = epoch_ms;
    s_ref.error_ms = NMEA_FIX_LATENCY_MS;
    s_ref.disciplined = false;  // a fresh PPS edge (if any) upgrades this in get_ref()
    s_ref.time_valid = true;
    s_ref.position_valid = has_pos;
    if (has_pos) {
        s_ref.lat_1e7 = lat_1e7;
        s_ref.lon_1e7 = lon_1e7;
    }
    portEXIT_CRITICAL(&s_lock);
    return true;
}

// -- UART line reader task ----------------------------------------------

static void gps_uart_task(void *arg) {
    (void)arg;
    char line[NMEA_LINE_MAX];
    size_t line_len = 0;
    uint8_t byte;

    while (1) {
        int n = uart_read_bytes(GPS_UART_NUM, &byte, 1, pdMS_TO_TICKS(1000));
        if (n <= 0) continue;
        if (byte == '\n' || byte == '\r') {
            if (line_len > 6 && line[0] == '$' && line[3] == 'R' &&
                line[4] == 'M' && line[5] == 'C') {
                line[line_len] = '\0';
                if (!parse_rmc(line)) {
                    ESP_LOGD(TAG, "RMC sentence present but no fix or unparsable");
                }
            }
            line_len = 0;
            continue;
        }
        if (line_len + 1 < sizeof(line)) {
            line[line_len++] = (char)byte;
        } else {
            line_len = 0;  // overlong garbage line — drop it and resync on the next newline
        }
    }
}

esp_err_t swarm_gps_time_start(void) {
    uart_config_t cfg = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(GPS_UART_NUM, 512, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(GPS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(GPS_UART_NUM, SWARM_PIN_GPS_TX, SWARM_PIN_GPS_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    if (swarm_board_has_pps()) {
        // SWARM_PIN_GPS_PPS >= 0 is exactly swarm_board_has_pps(), but the
        // shift is written this way (rather than 1ULL << SWARM_PIN_GPS_PPS
        // directly) so the boards where the macro is -1 never even
        // constant-fold a negative shift count into the build.
        uint64_t pps_bit = (SWARM_PIN_GPS_PPS >= 0) ? (1ULL << (unsigned)SWARM_PIN_GPS_PPS) : 0;
        gpio_config_t pps_cfg = {
            .pin_bit_mask = pps_bit,
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE,
        };
        gpio_config(&pps_cfg);
        // The radio driver (RadioLib's ESP-IDF HAL) installs the GPIO ISR service
        // during lora_radio_init(); a second gpio_install_isr_service() makes the
        // driver log an ERROR. Add the handler first and install only if no
        // service exists (a radio-less build), with the same IRAM flag the HAL uses.
        esp_err_t pps_err = gpio_isr_handler_add(SWARM_PIN_GPS_PPS, pps_isr, NULL);
        if (pps_err == ESP_ERR_INVALID_STATE) {
            pps_err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
            if (pps_err == ESP_OK) pps_err = gpio_isr_handler_add(SWARM_PIN_GPS_PPS, pps_isr, NULL);
        }
        if (pps_err != ESP_OK) ESP_LOGE(TAG, "PPS ISR not armed: %s", esp_err_to_name(pps_err));
        ESP_LOGI(TAG, "PPS ISR armed on GPIO %d", SWARM_PIN_GPS_PPS);
    } else {
        ESP_LOGW(TAG, "no PPS pin on this board — permanent NMEA fallback (ADR 0027)");
    }

    // Core 0, real-time priority: GPS discipline feeds the TDMA slot timer
    // (ADR 0028 decision 3), just below tdma_task itself.
    BaseType_t ok = xTaskCreatePinnedToCore(gps_uart_task, "gps_uart", 3072,
                                             NULL, 18, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t swarm_gps_time_get_ref(swarm_time_ref_t *out) {
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    swarm_time_ref_t snap;
    int64_t pps_us;
    portENTER_CRITICAL(&s_lock);
    snap = s_ref;
    pps_us = s_pps_boot_us;
    portEXIT_CRITICAL(&s_lock);

    if (!snap.time_valid) {
        *out = snap;
        return ESP_OK;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - snap.boot_us;
    int64_t elapsed_ms = elapsed_us / 1000;

    out->boot_us = now_us;
    out->epoch_ms = snap.epoch_ms + elapsed_ms;
    out->time_valid = true;
    out->position_valid = snap.position_valid;
    out->lat_1e7 = snap.lat_1e7;
    out->lon_1e7 = snap.lon_1e7;

    uint32_t drift = (uint32_t)((elapsed_ms / 60000) * DRIFT_MS_PER_MIN);
    uint32_t error_ms = NMEA_FIX_LATENCY_MS + drift;

    // A PPS edge in roughly the last second tightens the estimate: align
    // epoch_ms's sub-second part to the edge and report PPS-quality error.
    // See gps_time.h and ADR 0027 decision 6 — this is a bring-up stretch
    // path, not the plan of record for classic satellites.
    if (pps_us >= 0 && (now_us - pps_us) < (PPS_FRESH_WINDOW_MS * 1000)) {
        int64_t pps_epoch_ms = snap.epoch_ms + (pps_us - snap.boot_us) / 1000;
        int64_t aligned_second_ms = (pps_epoch_ms / 1000) * 1000;
        out->epoch_ms = aligned_second_ms + (out->epoch_ms - aligned_second_ms) % 1000;
        out->disciplined = true;
        error_ms = 5;
    } else {
        out->disciplined = false;
    }

    out->error_ms = error_ms > MAX_ERROR_MS ? MAX_ERROR_MS : error_ms;
    return ESP_OK;
}
