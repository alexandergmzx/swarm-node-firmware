/* SPDX-License-Identifier: MIT
 *
 * radio_lab — field radio tests on the T-Beam S3 Supreme.
 *
 * Modes (BOOT button, short press, or the `mode` command; the board boots in
 * LISTEN, which never transmits):
 *   PING    send a tagged PING, wait for the PONG, log loss, RSSI/SNR both
 *           ways and the GPS distance between the two boards (the base)
 *   PONG    answer every PING with this board's RSSI/SNR of it and its GPS
 *           position (the walker)
 *   LISTEN  print every packet heard at the current settings (other radios)
 *   SWEEP   RSSI noise floor across the band, bar graph on the screen
 * Presets (BOOT long press, or `preset N`): SF and TX power for range steps;
 * both boards must show the same preset. `set ...` gives free settings for
 * matching other LoRa radios.
 *
 * Rules held by construction:
 *   - every transmission obeys the profile's airtime ceiling
 *     (CONFIG_SWARM_RADIO_MAX_DUTY_PERMILLE: 1 % at 915-MX): the next TX may
 *     start only after time_on_air x 1000 / permille has passed;
 *   - transmit only on settings the profile's band allows (915-MX: a 500 kHz
 *     channel wholly inside 902-928 MHz, at most +22 dBm; 433-LAB: inside
 *     433.050-434.790 MHz, at most +10 dBm). Listening is allowed anywhere
 *     the chip tunes.
 * Radio work runs on Core 0; button, commands and screen on Core 1.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_pins.h"
#include "display.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_time.h"
#include "lora_radio.h"
#include "pmu.h"
#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "radio_lab is for the T-Beam S3 Supreme (esp32s3, SX1262)"
#endif

static const char *TAG = "radio_lab";

#define BUTTON_GPIO 0          /* BOOT: free as an input once the app runs */
#define LONG_PRESS_MS 800
#define DUTY_PERMILLE CONFIG_SWARM_RADIO_MAX_DUTY_PERMILLE

/* ---- band rules of the build profile ---- */
#if CONFIG_SWARM_RADIO_PROFILE_915
#define BAND_LO_KHZ 902000u
#define BAND_HI_KHZ 928000u
#define BAND_MAX_DBM 22
#define BAND_TX_BW_KHZ 500 /* IFT-008-2015 4.3.3: fixed channel >= 500 kHz */
#else
#define BAND_LO_KHZ 433050u
#define BAND_HI_KHZ 434790u
#define BAND_MAX_DBM 10
#define BAND_TX_BW_KHZ 0 /* any */
#endif

/* ---- the over-the-air test frame (18 B) ---- */
enum { FT_PING = 1, FT_PONG = 2, FT_BURST = 3 };
typedef struct __attribute__((packed)) {
    uint8_t magic[2];  /* 'S','L' */
    uint8_t type;
    uint16_t seq;
    uint8_t preset;     /* 0xFF = custom settings */
    int8_t tx_dbm;
    uint8_t flags;      /* bit 0: lat/lon valid */
    int32_t lat_1e7;
    int32_t lon_1e7;
    int8_t rssi_fwd;    /* PONG: RSSI of the PING at the responder, dBm */
    int8_t snr_fwd_q4;  /* PONG: SNR of the PING at the responder, dB x 4 */
} lab_frame_t;
_Static_assert(sizeof(lab_frame_t) == 18, "lab frame is 18 bytes");

/* ---- presets: same channel, rising SF and power ---- */
typedef struct {
    uint8_t sf;
    int8_t dbm;
} preset_t;
static const preset_t k_presets[] = {{8, 2}, {8, 14}, {8, 20}, {10, 20}, {12, 20}};
#define N_PRESETS (int)(sizeof(k_presets) / sizeof(k_presets[0]))

/* LISTEN first: a board boots listening and transmits nothing until a mode
 * that sends is chosen (an antenna may not be fitted yet). */
typedef enum { MODE_LISTEN, MODE_PING, MODE_PONG, MODE_SWEEP, MODE_COUNT } lab_mode_t;
static const char *k_mode_name[] = {"LISTEN", "PING", "PONG", "SWEEP"};

/* ---- shared state (spinlock) ---- */
#define SWEEP_CH 52 /* 500 kHz steps across 26 MHz */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    lab_mode_t mode;
    int preset;            /* -1 = custom */
    lora_params_t p;
    bool tx_allowed;       /* current settings pass the band rule */
    bool reconfigure;      /* settings or mode changed, radio task applies */
    uint32_t burst_left;   /* `tx N` frames still to send (any mode but SWEEP) */
    /* stats */
    uint32_t sent, back, heard, replied, skipped_duty, rx_any, bursts_sent;
    int last_rssi, last_snr_q4;        /* last frame we received */
    int fwd_rssi, fwd_snr_q4;          /* PONG's report of our PING */
    bool have_fwd;
    double dist_m;                     /* -1 = unknown */
    uint8_t last_len;
    uint8_t last_bytes[8];
    int64_t last_rx_us;
    uint32_t period_ms;
    float sweep_dbm[SWEEP_CH];
    int sweep_done;                    /* channels measured this pass */
} S = {.mode = MODE_LISTEN, .preset = 0, .dist_m = -1};

static bool band_tx_ok(const lora_params_t *p) {
    uint32_t half = p->bw_khz / 2u;
    if (BAND_TX_BW_KHZ && p->bw_khz != BAND_TX_BW_KHZ) return false;
    if (p->freq_khz < BAND_LO_KHZ + half || p->freq_khz > BAND_HI_KHZ - half) return false;
    return p->tx_dbm <= BAND_MAX_DBM && p->tx_dbm >= -9;
}

static void params_from_preset(int idx, lora_params_t *p) {
    const lora_profile_t *prof = lora_radio_profile();
    p->freq_khz = prof->freq_khz;
    p->bw_khz = prof->bw_khz;
    p->cr_denom = prof->cr_denom;
    p->sync_word = prof->sync_word;
    p->sf = k_presets[idx].sf;
    int8_t dbm = k_presets[idx].dbm;
    p->tx_dbm = dbm > BAND_MAX_DBM ? BAND_MAX_DBM : dbm;
}

/* ---- airtime ceiling ---- */
static int64_t s_next_tx_us = 0;

/* Transmit only if the ceiling allows it now; returns ESP_ERR_INVALID_STATE
 * when it does not (the caller waits or counts a skip). */
static esp_err_t tx_within_ceiling(const void *buf, size_t len) {
    int64_t now = esp_timer_get_time();
    if (now < s_next_tx_us) return ESP_ERR_INVALID_STATE;
    uint32_t toa = lora_radio_time_on_air_us(len);
    esp_err_t err = lora_radio_tx((const uint8_t *)buf, len);
    s_next_tx_us = now + (int64_t)toa * 1000 / DUTY_PERMILLE;
    return err;
}

static void wait_for_ceiling(void) {
    int64_t now = esp_timer_get_time();
    if (s_next_tx_us > now) vTaskDelay(pdMS_TO_TICKS((s_next_tx_us - now) / 1000 + 1));
}

/* ---- helpers ---- */
static double haversine_m(int32_t la1, int32_t lo1, int32_t la2, int32_t lo2) {
    const double r = 6371000.0, k = M_PI / 180.0 / 1e7;
    double p1 = la1 * k, p2 = la2 * k, dp = (la2 - la1) * k, dl = (lo2 - lo1) * k;
    double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2 * r * atan2(sqrt(a), sqrt(1 - a));
}

static void fill_frame(lab_frame_t *f, uint8_t type, uint16_t seq, int preset, int8_t dbm) {
    memset(f, 0, sizeof(*f));
    f->magic[0] = 'S';
    f->magic[1] = 'L';
    f->type = type;
    f->seq = seq;
    f->preset = preset < 0 ? 0xFF : (uint8_t)preset;
    f->tx_dbm = dbm;
    swarm_time_ref_t ref;
    if (swarm_gps_time_get_ref(&ref) == ESP_OK && ref.position_valid) {
        f->flags = 1;
        f->lat_1e7 = ref.lat_1e7;
        f->lon_1e7 = ref.lon_1e7;
    }
}

static bool parse_frame(const uint8_t *b, size_t n, lab_frame_t *out) {
    if (n != sizeof(lab_frame_t) || b[0] != 'S' || b[1] != 'L') return false;
    memcpy(out, b, sizeof(*out));
    return true;
}

static void note_rx(const uint8_t *buf, size_t n, float rssi, float snr) {
    portENTER_CRITICAL(&s_lock);
    S.rx_any++;
    S.last_rssi = (int)lrintf(rssi);
    S.last_snr_q4 = (int)lrintf(snr * 4);
    S.last_len = (uint8_t)(n > 255 ? 255 : n);
    memcpy(S.last_bytes, buf, n < 8 ? n : 8);
    S.last_rx_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

static void print_rx_line(const char *tag, const uint8_t *buf, size_t n, float rssi, float snr) {
    static const char hex[] = "0123456789abcdef";
    char h[2 * 64 + 1];
    size_t m = n < 64 ? n : 64;
    for (size_t i = 0; i < m; i++) {
        h[2 * i] = hex[buf[i] >> 4];
        h[2 * i + 1] = hex[buf[i] & 15];
    }
    h[2 * m] = '\0';
    printf("%s,%lld,%.1f,%.2f,%u,%s\n", tag, (long long)(esp_timer_get_time() / 1000), rssi, snr,
           (unsigned)n, h);
}

static void my_position(bool *valid, int32_t *lat, int32_t *lon) {
    swarm_time_ref_t ref;
    *valid = swarm_gps_time_get_ref(&ref) == ESP_OK && ref.position_valid;
    *lat = ref.lat_1e7;
    *lon = ref.lon_1e7;
}

/* ---- radio task (Core 0) ---- */
static lora_params_t s_applied;
static lab_mode_t s_applied_mode = MODE_COUNT;

static void apply_if_needed(void) {
    portENTER_CRITICAL(&s_lock);
    bool need = S.reconfigure || S.mode != s_applied_mode;
    lora_params_t p = S.p;
    lab_mode_t mode = S.mode;
    S.reconfigure = false;
    portEXIT_CRITICAL(&s_lock);
    if (!need) return;
    if (lora_radio_set_params(&p) != ESP_OK) {
        printf("ERR,settings rejected by the radio; keeping the previous ones\n");
        portENTER_CRITICAL(&s_lock);
        S.p = s_applied;
        portEXIT_CRITICAL(&s_lock);
        lora_radio_set_params(&s_applied);
        p = s_applied;
    }
    uint32_t toa = lora_radio_time_on_air_us(sizeof(lab_frame_t));
    uint32_t period = (uint32_t)((uint64_t)toa * 1000 / DUTY_PERMILLE / 1000) + 50; /* ms, a margin above the floor */
    if (period < 1000) period = 1000;
    portENTER_CRITICAL(&s_lock);
    S.tx_allowed = band_tx_ok(&p);
    S.period_ms = period;
    if (mode != s_applied_mode) {
        S.sent = S.back = S.heard = S.replied = S.skipped_duty = S.rx_any = S.bursts_sent = 0;
        S.have_fwd = false;
        S.dist_m = -1;
        S.sweep_done = 0;
    }
    portEXIT_CRITICAL(&s_lock);
    s_applied = p;
    s_applied_mode = mode;
    printf("CFG,%s,%lu,%u,%u,%u,0x%02x,%d,toa_us=%lu,period_ms=%lu,tx=%s\n", k_mode_name[mode],
           (unsigned long)p.freq_khz, p.bw_khz, p.sf, p.cr_denom, p.sync_word, p.tx_dbm,
           (unsigned long)toa, (unsigned long)period, band_tx_ok(&p) ? "allowed" : "BLOCKED (band rule)");
    lora_radio_rx_start();
}

static void send_burst_if_any(int preset) {
    portENTER_CRITICAL(&s_lock);
    bool go = S.burst_left > 0 && S.tx_allowed;
    portEXIT_CRITICAL(&s_lock);
    if (!go) return;
    static uint16_t seq;
    if (esp_timer_get_time() < s_next_tx_us) return;  /* not yet: keep seq consecutive */
    lab_frame_t f;
    fill_frame(&f, FT_BURST, (uint16_t)(seq + 1), preset, s_applied.tx_dbm);
    if (tx_within_ceiling(&f, sizeof(f)) == ESP_OK) {
        seq++;
        printf("BURST,%lld,%u\n", (long long)(esp_timer_get_time() / 1000), seq);
        portENTER_CRITICAL(&s_lock);
        S.burst_left--;
        S.bursts_sent++;
        portEXIT_CRITICAL(&s_lock);
    }
    lora_radio_rx_start();
}

static void do_ping(void) {
    static uint16_t seq;
    uint8_t buf[64];
    size_t n;
    float rssi, snr;
    portENTER_CRITICAL(&s_lock);
    bool allowed = S.tx_allowed;
    int preset = S.preset;
    uint32_t period = S.period_ms;
    portEXIT_CRITICAL(&s_lock);
    int64_t cycle_start = esp_timer_get_time();
    if (allowed) {
        wait_for_ceiling();
        lab_frame_t f;
        fill_frame(&f, FT_PING, ++seq, preset, s_applied.tx_dbm);
        if (tx_within_ceiling(&f, sizeof(f)) == ESP_OK) {
            portENTER_CRITICAL(&s_lock);
            S.sent++;
            portEXIT_CRITICAL(&s_lock);
        }
        lora_radio_rx_start();
    }
    /* Reply window: the PONG's airtime plus generous turnaround. */
    uint32_t window_ms = 2 * lora_radio_time_on_air_us(sizeof(lab_frame_t)) / 1000 + 400;
    bool got = false;
    while ((esp_timer_get_time() - cycle_start) / 1000 < (int64_t)period) {
        if (lora_radio_rx_poll(buf, sizeof(buf), &n, &rssi, &snr, 100) != ESP_OK) {
            if (!got && (esp_timer_get_time() - cycle_start) / 1000 > window_ms) break;
            continue;
        }
        note_rx(buf, n, rssi, snr);
        lab_frame_t r;
        if (allowed && !got && parse_frame(buf, n, &r) && r.type == FT_PONG && r.seq == seq) {
            got = true;
            bool mine;
            int32_t lat, lon;
            my_position(&mine, &lat, &lon);
            double d = (mine && (r.flags & 1)) ? haversine_m(lat, lon, r.lat_1e7, r.lon_1e7) : -1;
            portENTER_CRITICAL(&s_lock);
            S.back++;
            S.fwd_rssi = r.rssi_fwd;
            S.fwd_snr_q4 = r.snr_fwd_q4;
            S.have_fwd = true;
            S.dist_m = d;
            uint32_t sent = S.sent, back = S.back;
            portEXIT_CRITICAL(&s_lock);
            printf("PP,%lld,%u,%d,%u,%d,ok,%d,%.2f,%.1f,%.2f,%.1f,%ld,%ld,%ld,%ld,%lu,%lu\n",
                   (long long)(esp_timer_get_time() / 1000), seq, preset, s_applied.sf, s_applied.tx_dbm,
                   r.rssi_fwd, r.snr_fwd_q4 / 4.0, rssi, snr, d, (long)lat, (long)lon, (long)r.lat_1e7,
                   (long)r.lon_1e7, (unsigned long)sent, (unsigned long)back);
        } else {
            print_rx_line("RX", buf, n, rssi, snr);
        }
    }
    if (allowed && !got) {
        printf("PP,%lld,%u,%d,%u,%d,lost\n", (long long)(esp_timer_get_time() / 1000), seq, preset,
               s_applied.sf, s_applied.tx_dbm);
    }
    send_burst_if_any(preset);
    /* Hold the cycle to the period so the ceiling, not the loop, sets the pace. */
    int64_t left_ms = (int64_t)period - (esp_timer_get_time() - cycle_start) / 1000;
    if (left_ms > 0) {
        while (left_ms > 0 && lora_radio_rx_poll(buf, sizeof(buf), &n, &rssi, &snr, (uint32_t)left_ms) == ESP_OK) {
            note_rx(buf, n, rssi, snr);
            print_rx_line("RX", buf, n, rssi, snr);
            left_ms = (int64_t)period - (esp_timer_get_time() - cycle_start) / 1000;
        }
    }
}

static void do_pong(void) {
    uint8_t buf[64];
    size_t n;
    float rssi, snr;
    if (lora_radio_rx_poll(buf, sizeof(buf), &n, &rssi, &snr, 200) != ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        int preset = S.preset;
        portEXIT_CRITICAL(&s_lock);
        send_burst_if_any(preset);
        return;
    }
    note_rx(buf, n, rssi, snr);
    lab_frame_t r;
    if (!parse_frame(buf, n, &r) || r.type != FT_PING) {
        print_rx_line("RX", buf, n, rssi, snr);
        return;
    }
    bool mine;
    int32_t lat, lon;
    my_position(&mine, &lat, &lon);
    double d = (mine && (r.flags & 1)) ? haversine_m(lat, lon, r.lat_1e7, r.lon_1e7) : -1;
    portENTER_CRITICAL(&s_lock);
    S.heard++;
    S.dist_m = d;
    bool allowed = S.tx_allowed;
    int preset = S.preset;
    portEXIT_CRITICAL(&s_lock);
    const char *what = "heard";
    if (allowed) {
        lab_frame_t f;
        fill_frame(&f, FT_PONG, r.seq, preset, s_applied.tx_dbm);
        f.rssi_fwd = (int8_t)lrintf(rssi < -128 ? -128 : rssi);
        f.snr_fwd_q4 = (int8_t)lrintf(snr * 4 < -128 ? -128 : snr * 4 > 127 ? 127 : snr * 4);
        vTaskDelay(pdMS_TO_TICKS(10)); /* let the pinger switch to RX */
        esp_err_t e = tx_within_ceiling(&f, sizeof(f));
        portENTER_CRITICAL(&s_lock);
        if (e == ESP_OK) S.replied++;
        else if (e == ESP_ERR_INVALID_STATE) S.skipped_duty++;
        portEXIT_CRITICAL(&s_lock);
        what = e == ESP_OK ? "replied" : e == ESP_ERR_INVALID_STATE ? "skipped_ceiling" : "tx_error";
        lora_radio_rx_start();
    }
    printf("PO,%lld,%u,%u,%d,%.1f,%.2f,%s,%.1f,%ld,%ld,%ld,%ld\n", (long long)(esp_timer_get_time() / 1000),
           r.seq, r.preset, r.tx_dbm, rssi, snr, what, d, (long)lat, (long)lon, (long)r.lat_1e7, (long)r.lon_1e7);
}

static void do_listen(void) {
    uint8_t buf[256];
    size_t n;
    float rssi, snr;
    if (lora_radio_rx_poll(buf, sizeof(buf), &n, &rssi, &snr, 200) == ESP_OK) {
        note_rx(buf, n, rssi, snr);
        print_rx_line("RX", buf, n, rssi, snr);
    }
    portENTER_CRITICAL(&s_lock);
    int preset = S.preset;
    portEXIT_CRITICAL(&s_lock);
    send_burst_if_any(preset);
}

static void do_sweep(void) {
    lora_params_t p = s_applied;
    uint32_t bw = 500, lo = BAND_LO_KHZ + bw / 2;
    for (int ch = 0; ch < SWEEP_CH; ch++) {
        portENTER_CRITICAL(&s_lock);
        bool leave = S.mode != MODE_SWEEP || S.reconfigure;
        portEXIT_CRITICAL(&s_lock);
        if (leave) break;
        p.freq_khz = lo + (uint32_t)ch * bw;
        if (p.freq_khz + bw / 2 > BAND_HI_KHZ) break;
        p.bw_khz = (uint16_t)bw;
        if (lora_radio_set_params(&p) != ESP_OK) continue;
        lora_radio_rx_start();
        vTaskDelay(pdMS_TO_TICKS(4));
        float sum = 0, peak = -200;
        for (int i = 0; i < 16; i++) {
            float v = -200;
            lora_radio_rssi_now(&v);
            sum += v;
            if (v > peak) peak = v;
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        float mean = sum / 16;
        printf("SW,%lld,%lu,%.1f,%.1f\n", (long long)(esp_timer_get_time() / 1000), (unsigned long)p.freq_khz,
               mean, peak);
        portENTER_CRITICAL(&s_lock);
        S.sweep_dbm[ch] = peak;
        S.sweep_done = ch + 1;
        portEXIT_CRITICAL(&s_lock);
    }
    /* back to the configured channel before the next pass or mode */
    lora_radio_set_params(&s_applied);
    lora_radio_rx_start();
}

static void radio_task(void *arg) {
    (void)arg;
    while (1) {
        apply_if_needed();
        switch (s_applied_mode) {
        case MODE_PING: do_ping(); break;
        case MODE_PONG: do_pong(); break;
        case MODE_LISTEN: do_listen(); break;
        case MODE_SWEEP: do_sweep(); break;
        default: vTaskDelay(pdMS_TO_TICKS(100)); break;
        }
    }
}

/* ---- controls (Core 1) ---- */
static void set_mode(lab_mode_t m) {
    portENTER_CRITICAL(&s_lock);
    S.mode = m;
    portEXIT_CRITICAL(&s_lock);
}

static void set_preset(int idx) {
    lora_params_t p;
    params_from_preset(idx, &p);
    portENTER_CRITICAL(&s_lock);
    S.preset = idx;
    S.p = p;
    S.reconfigure = true;
    portEXIT_CRITICAL(&s_lock);
}

static void button_task(void *arg) {
    (void)arg;
    gpio_config_t cfg = {.pin_bit_mask = 1ULL << BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&cfg);
    int64_t down_us = -1;
    while (1) {
        bool pressed = gpio_get_level(BUTTON_GPIO) == 0;
        int64_t now = esp_timer_get_time();
        if (pressed && down_us < 0) down_us = now;
        if (!pressed && down_us >= 0) {
            int64_t held = (now - down_us) / 1000;
            down_us = -1;
            if (held >= LONG_PRESS_MS) {
                portENTER_CRITICAL(&s_lock);
                int next = (S.preset + 1) % N_PRESETS;
                portEXIT_CRITICAL(&s_lock);
                set_preset(next < 0 ? 0 : next);
            } else if (held >= 30) {
                portENTER_CRITICAL(&s_lock);
                lab_mode_t next = (lab_mode_t)((S.mode + 1) % MODE_COUNT);
                portEXIT_CRITICAL(&s_lock);
                set_mode(next);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void print_help(void) {
    printf("radio_lab commands:\n"
           "  mode listen|ping|pong|sweep   (BOOT short press cycles; boots in listen)\n"
           "  preset 0-%d                   (BOOT long press cycles; SF/dBm:", N_PRESETS - 1);
    for (int i = 0; i < N_PRESETS; i++) printf(" %d=SF%u/%+d", i, k_presets[i].sf, k_presets[i].dbm);
    printf(")\n"
           "  set freq <MHz> | bw <kHz> | sf <5-12> | cr <5-8> | sync <hex> | pwr <dBm>\n"
           "  tx <N>       send N tagged burst frames, paced to the airtime ceiling\n"
           "  status | help\n"
           "Output lines (CSV): CFG, PP (ping result), PO (pong), RX (any packet), SW (sweep), BURST\n");
}

static void print_status(void) {
    portENTER_CRITICAL(&s_lock);
    typeof(S) c = S;
    portEXIT_CRITICAL(&s_lock);
    printf("STATUS,%s,preset=%d,%lu kHz,%u kHz,SF%u,CR4:%u,sync 0x%02x,%+d dBm,tx=%s,period=%lu ms,"
           "sent=%lu,back=%lu,heard=%lu,replied=%lu,skipped=%lu,rx=%lu,dist=%.1f\n",
           k_mode_name[c.mode], c.preset, (unsigned long)c.p.freq_khz, c.p.bw_khz, c.p.sf, c.p.cr_denom,
           c.p.sync_word, c.p.tx_dbm, c.tx_allowed ? "allowed" : "blocked", (unsigned long)c.period_ms,
           (unsigned long)c.sent, (unsigned long)c.back, (unsigned long)c.heard, (unsigned long)c.replied,
           (unsigned long)c.skipped_duty, (unsigned long)c.rx_any, c.dist_m);
}

static void handle_line(char *line) {
    char *cmd = strtok(line, " \t\r\n");
    if (!cmd) return;
    char *a = strtok(NULL, " \t\r\n");
    char *b = strtok(NULL, " \t\r\n");
    if (!strcmp(cmd, "help")) { print_help(); return; }
    if (!strcmp(cmd, "status")) { print_status(); return; }
    if (!strcmp(cmd, "mode") && a) {
        for (int m = 0; m < MODE_COUNT; m++) {
            char low[8];
            snprintf(low, sizeof(low), "%s", k_mode_name[m]);
            for (char *q = low; *q; q++) *q = (char)(*q + ('a' - 'A') * (*q >= 'A' && *q <= 'Z'));
            if (!strcmp(a, low)) { set_mode((lab_mode_t)m); return; }
        }
    } else if (!strcmp(cmd, "preset") && a) {
        int i = atoi(a);
        if (i >= 0 && i < N_PRESETS) { set_preset(i); return; }
    } else if (!strcmp(cmd, "tx") && a) {
        portENTER_CRITICAL(&s_lock);
        S.burst_left = (uint32_t)atoi(a);
        portEXIT_CRITICAL(&s_lock);
        return;
    } else if (!strcmp(cmd, "set") && a && b) {
        portENTER_CRITICAL(&s_lock);
        lora_params_t p = S.p;
        portEXIT_CRITICAL(&s_lock);
        bool ok = true;
        if (!strcmp(a, "freq")) p.freq_khz = (uint32_t)lrint(atof(b) * 1000.0);
        else if (!strcmp(a, "bw")) p.bw_khz = (uint16_t)atoi(b);
        else if (!strcmp(a, "sf")) p.sf = (uint8_t)atoi(b);
        else if (!strcmp(a, "cr")) p.cr_denom = (uint8_t)atoi(b);
        else if (!strcmp(a, "sync")) p.sync_word = (uint8_t)strtoul(b, NULL, 16);
        else if (!strcmp(a, "pwr")) p.tx_dbm = (int8_t)atoi(b);
        else ok = false;
        if (ok) {
            portENTER_CRITICAL(&s_lock);
            S.p = p;
            S.preset = -1;
            S.reconfigure = true;
            portEXIT_CRITICAL(&s_lock);
            if (!band_tx_ok(&p)) printf("NOTE,these settings are listen-only: transmit needs a %s\n",
                                        BAND_TX_BW_KHZ ? "500 kHz channel inside 902-928 MHz at <= +22 dBm"
                                                       : "channel inside 433.050-434.790 MHz at <= +10 dBm");
            return;
        }
    }
    printf("ERR,unknown command (type help)\n");
}

static void console_task(void *arg) {
    (void)arg;
    char line[96];
    size_t len = 0;
    while (1) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(100)) <= 0) continue;
        if (c == '\n' || c == '\r') {
            if (len) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        } else if (len + 1 < sizeof(line)) {
            line[len++] = (char)c;
        }
    }
}

/* ---- screen (Core 1, 1 Hz) ---- */
static void render(void) {
    portENTER_CRITICAL(&s_lock);
    typeof(S) c = S;
    portEXIT_CRITICAL(&s_lock);
    char pr[12];
    if (c.preset < 0) snprintf(pr, sizeof(pr), "CUST");
    else snprintf(pr, sizeof(pr), "P%d", c.preset);
    swarm_display_row(0, "%s %s SF%u %+ddBm", k_mode_name[c.mode], pr, c.p.sf, c.p.tx_dbm);
    swarm_display_row(1, "%lu.%03lu %uk %s", (unsigned long)(c.p.freq_khz / 1000),
                      (unsigned long)(c.p.freq_khz % 1000), c.p.bw_khz, c.tx_allowed ? "" : "RX ONLY");
    swarm_time_ref_t ref;
    bool fix = swarm_gps_time_get_ref(&ref) == ESP_OK && ref.position_valid;
    char dist[22];
    if (c.dist_m >= 0) snprintf(dist, sizeof(dist), "dist %.0f m", c.dist_m);
    else snprintf(dist, sizeof(dist), "dist: %s", fix ? "peer has no fix" : "no GPS fix");
    long age = c.last_rx_us > 0 ? (long)((esp_timer_get_time() - c.last_rx_us) / 1000000) : -1;
    switch (c.mode) {
    case MODE_PING: {
        uint32_t lost = c.sent > c.back ? c.sent - c.back : 0;
        swarm_display_row(2, "sent %lu back %lu", (unsigned long)c.sent, (unsigned long)c.back);
        swarm_display_row(3, "loss %lu%%", c.sent ? (unsigned long)(100 * lost / c.sent) : 0UL);
        if (c.have_fwd) swarm_display_row(4, "fwd %ddBm %.1fdB", c.fwd_rssi, c.fwd_snr_q4 / 4.0);
        else swarm_display_row(4, "fwd --");
        if (c.back) swarm_display_row(5, "bck %ddBm %.1fdB %lds", c.last_rssi, c.last_snr_q4 / 4.0, age);
        else swarm_display_row(5, "bck --");
        swarm_display_row(6, "%s", dist);
        swarm_display_row(7, "every %.1fs air<=%.0f%%", c.period_ms / 1000.0, DUTY_PERMILLE / 10.0);
        break;
    }
    case MODE_PONG:
        swarm_display_row(2, "heard %lu", (unsigned long)c.heard);
        swarm_display_row(3, "replied %lu skip %lu", (unsigned long)c.replied, (unsigned long)c.skipped_duty);
        if (c.heard) swarm_display_row(4, "rx %ddBm %.1fdB %lds", c.last_rssi, c.last_snr_q4 / 4.0, age);
        else swarm_display_row(4, "waiting for PING");
        swarm_display_row(6, "%s", dist);
        swarm_display_row(7, "%s", fix ? "GPS fix" : "GPS no fix");
        break;
    case MODE_LISTEN:
        swarm_display_row(2, "packets %lu", (unsigned long)c.rx_any);
        if (c.rx_any) {
            swarm_display_row(3, "%ddBm %.1fdB %uB %lds", c.last_rssi, c.last_snr_q4 / 4.0, c.last_len, age);
            swarm_display_row(4, "%02x%02x%02x%02x %02x%02x%02x%02x", c.last_bytes[0], c.last_bytes[1],
                              c.last_bytes[2], c.last_bytes[3], c.last_bytes[4], c.last_bytes[5],
                              c.last_bytes[6], c.last_bytes[7]);
        }
        swarm_display_row(5, "sync 0x%02x CR4:%u", c.p.sync_word, c.p.cr_denom);
        if (c.bursts_sent || c.burst_left)
            swarm_display_row(6, "burst %lu left %lu", (unsigned long)c.bursts_sent, (unsigned long)c.burst_left);
        swarm_display_row(7, "%s", fix ? "GPS fix" : "GPS no fix");
        break;
    case MODE_SWEEP: {
        /* rows 1-6 become a bar graph: -130 dBm at the bottom, -60 at the top */
        float lo = 0, hi = -200;
        int at = 0;
        for (int i = 0; i < c.sweep_done; i++) {
            if (c.sweep_dbm[i] > hi) { hi = c.sweep_dbm[i]; at = i; }
            if (i == 0 || c.sweep_dbm[i] < lo) lo = c.sweep_dbm[i];
        }
        swarm_display_row(1, " ");
        swarm_display_row(7, c.sweep_done ? "min%.0f max%.0f@%.1f" : "sweeping...", lo, hi,
                          (BAND_LO_KHZ + 250 + at * 500) / 1000.0);
        for (int i = 0; i < c.sweep_done; i++) {
            float v = c.sweep_dbm[i];
            int h = (int)((v + 130.0f) * 47.0f / 70.0f);
            if (h < 1) h = 1;
            if (h > 47) h = 47;
            swarm_display_vbar(2 + i * 2, 1, 55 - h, 55);
        }
        break;
    }
    default: break;
    }
}

void app_main(void) {
    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&ucfg);
    usb_serial_jtag_vfs_use_driver();

    swarm_pmu_info_t pmu = {0};
    if (swarm_pmu_init(NULL, &pmu) != ESP_OK) ESP_LOGE(TAG, "PMU init failed");
    if (swarm_display_init(NULL) == ESP_OK) {
        swarm_display_row(0, "RADIO LAB booting");
        swarm_display_flush();
    }
    lora_chip_t chip;
    if (lora_radio_init(&chip) != ESP_OK || chip != LORA_CHIP_SX1262) {
        ESP_LOGE(TAG, "no SX1262 — radio_lab needs a T-Beam S3 Supreme");
        swarm_display_row(2, "NO SX1262 RADIO");
        swarm_display_flush();
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    swarm_gps_time_start();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("\n=== radio_lab on %02x:%02x:%02x:%02x:%02x:%02x — profile %s, airtime ceiling %.1f %% ===\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], lora_radio_profile()->name, DUTY_PERMILLE / 10.0);
    set_preset(0);
    print_help();

    if (swarm_display_present()) swarm_display_start(render, 1000);
    xTaskCreatePinnedToCore(radio_task, "lab_radio", 6144, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(button_task, "lab_button", 2560, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(console_task, "lab_console", 4096, NULL, 3, NULL, 1);
}
