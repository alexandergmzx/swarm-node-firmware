/* SPDX-License-Identifier: MIT
 *
 * M1 bring-up diagnostic (Step 4, ADR 0027/0028). Reports, once, what is
 * actually wired on a classic T-Beam or a T-Beam S3 Supreme: PMU identity and
 * rails, I2C scan, radio identity at the build's interop profile, and NMEA
 * (baud rate, plus PPS edges on a board that routes them — the classic does
 * not). CONFIG_BOARD_PROBE_LINK_TEST=n makes it a pass that never transmits.
 * Not the shipped satellite firmware; a throwaway
 * tool whose output goes verbatim into hardware/bringup-log.md.
 *
 * The AXP2101 is brought up first through components/pmu: the radio and the
 * GNSS module hang from its ALDO2/ALDO3 rails, off until enabled, so a
 * radio probe before that step would always report "no radio".
 */

#include <stdarg.h>
#include <stdio.h>

#include "board_pins.h"
#include "display.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_radio.h"
#include "pmu.h"

static const char *TAG = "board_probe";

#define I2C_XFER_TIMEOUT_MS 200

static volatile uint32_t s_pps_edges = 0;

/* The probe's results on the OLED, one row per step, redrawn as each step
 * finishes (CONFIG_SWARM_DISPLAY): the board shows its own verdict without a
 * laptop. Row 0 is the title bar. */
static char s_screen[SWARM_DISPLAY_ROWS][SWARM_DISPLAY_COLS + 1];

static void screen(int row, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void screen(int row, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_screen[row], sizeof(s_screen[row]), fmt, ap);
    va_end(ap);
    if (!swarm_display_present()) return;
    swarm_display_clear();
    for (int r = 0; r < SWARM_DISPLAY_ROWS; r++) swarm_display_row(r, "%s", s_screen[r]);
    swarm_display_flush();
}

static void IRAM_ATTR pps_isr(void *arg) {
    (void)arg;
    s_pps_edges++;
}

static i2c_master_bus_handle_t open_sensor_bus(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = SWARM_PIN_I2C_SDA,
        .scl_io_num = SWARM_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed on SDA=%d SCL=%d", SWARM_PIN_I2C_SDA, SWARM_PIN_I2C_SCL);
        return NULL;
    }
    return bus;
}

static void bring_up_pmu(i2c_master_bus_handle_t bus) {
    printf("--- PMU (AXP2101 expected — bench ID 2026-09-03) ---\n");
    swarm_pmu_info_t info = {0};
    esp_err_t err = swarm_pmu_init(bus, &info);
    if (err != ESP_OK) {
        screen(1, "PMU FAILED");
        printf("  PMU init FAILED (%s): nothing answered at 0x%02x as an AXP2101 — the radio and GNSS\n"
               "  rails are NOT enabled; every result below is expected to fail\n",
               esp_err_to_name(err), SWARM_I2C_ADDR_PMU);
        return;
    }
    printf("  chip id 0x%02x -> %s\n", info.chip_id,
           info.chip_id == 0x4A ? "AXP2101 (matches the 2026-09-03 bench identification)"
                                : "unrecognized (AXP2101 answers 0x4A)");
#if CONFIG_IDF_TARGET_ESP32
    printf("  rails enabled: ALDO2 (LoRa) + ALDO3 (GPS) at 3.3 V\n");
#else
    printf("  rails enabled: ALDO3 (radio) + ALDO4 (GNSS) at 3.3 V\n");
#endif
    screen(1, "PMU %s %.2fV", info.chip_id == 0x4A ? "AXP2101 ok" : "unknown", info.vbus_mv / 1000.0);
    printf("  battery %u mV (%s%s), vbus %u mV (%s), sys %u mV\n",
           info.batt_mv, info.battery_present ? "cell fitted" : "no cell",
           info.charging ? ", charging" : "", info.vbus_mv,
           info.vbus_present ? "present" : "absent", info.sys_mv);
}

static void scan_i2c(i2c_master_bus_handle_t bus) {
    printf("--- I2C scan (SDA=%d SCL=%d) ---\n", SWARM_PIN_I2C_SDA, SWARM_PIN_I2C_SCL);
    int acks = 0;
    bool kit = false;
    for (uint16_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, I2C_XFER_TIMEOUT_MS) == ESP_OK) {
            const char *known = "";
            if (addr == SWARM_I2C_ADDR_PMU) known = " (PMU AXP2101)";
#if CONFIG_IDF_TARGET_ESP32
            else if (addr == SWARM_I2C_ADDR_OLED) known = " (OLED — SSD1306 on the classics, fitted on both units)";
#else
            else if (addr == SWARM_I2C_ADDR_OLED) known = " (onboard OLED — SH1106; a QMC6310N magnetometer would also answer here)";
            else if (addr == 0x1c) known = " (magnetometer QMC6310U)";
            else if (addr == 0x77) known = " (onboard BME280)";
#endif
            else if (addr == SWARM_I2C_ADDR_SCD30) known = " (SCD30)";
            else if (addr == SWARM_I2C_ADDR_BME688) known = " (BME688)";
            printf("  0x%02x ACKed%s\n", addr, known);
            acks++;
            if (addr == SWARM_I2C_ADDR_SCD30 || addr == SWARM_I2C_ADDR_BME688) kit = true;
        }
    }
    screen(2, "I2C %d devices%s", acks, kit ? " +kit" : "");
#if CONFIG_IDF_TARGET_ESP32
    printf("  expected on a classic without the kit wired: 0x%02x, 0x%02x; with the kit: + 0x%02x, 0x%02x\n",
           SWARM_I2C_ADDR_PMU, SWARM_I2C_ADDR_OLED, SWARM_I2C_ADDR_SCD30, SWARM_I2C_ADDR_BME688);
#else
    printf("  expected on an S3 Supreme (PMU and RTC sit on the other bus, IO42/IO41): onboard OLED 0x%02x,\n"
           "  BME280 0x77, magnetometer 0x1c (QMC6310U; 0x3c QMC6310N, 0x7c QMC6309); the IMU is on SPI;\n"
           "  with the kit: + 0x%02x, 0x%02x\n",
           SWARM_I2C_ADDR_OLED, SWARM_I2C_ADDR_SCD30, SWARM_I2C_ADDR_BME688);
#endif
}

static void probe_radio(void) {
    const lora_profile_t *prof = lora_radio_profile();
    printf("--- LoRa radio probe (profile %s: %lu.%03lu MHz / %u kHz / SF%u / %+d dBm, RST=%d) ---\n",
           prof->name, (unsigned long)(prof->freq_khz / 1000), (unsigned long)(prof->freq_khz % 1000),
           prof->bw_khz, prof->sf, prof->tx_dbm, SWARM_PIN_LORA_RST);
    lora_chip_t chip = LORA_CHIP_UNKNOWN;
    esp_err_t err = lora_radio_init(&chip);
    if (err == ESP_OK) {
        printf("  found: %s\n", chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262");
        screen(3, "radio %s ok", chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262");
#if CONFIG_IDF_TARGET_ESP32
        if (chip == LORA_CHIP_SX1278) {
            printf("  (matches the HPDTek HPD14A on the purchase record; RST=%d confirmed)\n", SWARM_PIN_LORA_RST);
        } else {
            printf("  (an SX1262 on a classic contradicts the purchase record — note it in hardware/bringup-log.md)\n");
        }
#endif
    } else {
        screen(3, "NO RADIO");
        printf("  no radio responded (%s) — with the rails on and RST=%d this contradicts the vendor\n"
               "  schematic: check the PMU section above first, then SPI wiring\n",
               esp_err_to_name(err), SWARM_PIN_LORA_RST);
    }
}

static void echo_nmea_and_count_pps(void) {
    printf("--- GPS: NMEA echo + PPS presence (10 s window) ---\n");

    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_1, 512, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &cfg);
    uart_set_pin(UART_NUM_1, SWARM_PIN_GPS_TX, SWARM_PIN_GPS_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (swarm_board_has_pps()) {
        // See the identical guard in gps_time.c: written this way so a
        // board with SWARM_PIN_GPS_PPS == -1 never constant-folds a
        // negative/overflowing shift count into the build.
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
    } else {
        printf("  no PPS pin on this board by design (TIME_PULSE drives only the fix LED) — no edge count.\n");
    }

    int64_t start = esp_timer_get_time();
    char line[96];
    size_t len = 0;
    int lines_seen = 0;
    while ((esp_timer_get_time() - start) < 10 * 1000 * 1000) {
        uint8_t byte;
        if (uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(200)) > 0) {
            if (byte == '\n' || byte == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    printf("  NMEA: %s\n", line);
                    lines_seen++;
                    len = 0;
                }
            } else if (len + 1 < sizeof(line)) {
                line[len++] = (char)byte;
            }
        }
    }
    printf("  %d NMEA line(s) seen in 10 s", lines_seen);
    screen(4, "NMEA %d/10s PPS %u", lines_seen, (unsigned)s_pps_edges);
    if (swarm_board_has_pps()) {
        printf("; %u PPS edge(s)\n", (unsigned)s_pps_edges);
    } else {
        printf("\n");
    }
    if (lines_seen == 0) {
        // gps_time assumes 9600 (the L76K and MAX-M10S factory default); a
        // module whose saved configuration says otherwise shows up here.
        static const int bauds[] = {38400, 115200, 4800};
        for (size_t b = 0; b < sizeof(bauds) / sizeof(bauds[0]); b++) {
            uart_set_baudrate(UART_NUM_1, bauds[b]);
            uart_flush_input(UART_NUM_1);
            int dollars = 0;
            int64_t t0 = esp_timer_get_time();
            while ((esp_timer_get_time() - t0) < 3 * 1000 * 1000) {
                uint8_t byte;
                if (uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(200)) > 0 && byte == '$') dollars++;
            }
            printf("  baud scan: %d '$' sentence starts in 3 s at %d baud\n", dollars, bauds[b]);
        }
        printf("  no NMEA at 9600 — with the GNSS rail on this contradicts the schematic: check GPS_RX=%d/GPS_TX=%d\n",
               SWARM_PIN_GPS_RX, SWARM_PIN_GPS_TX);
    }
}

/* Bench / range link test: sixty tagged frames, one per second, at the
 * configured TX power (CONFIG_SWARM_TX_POWER_DBM, default +2 dBm; ADR 0031). The receiver prints each as
 * `RX <rssi> <snr> <hex>`; the hex starts with a5 5a 4c 4e 4b ("LNK") then
 * the sequence number and this chip's MAC tail, so the two consoles can be
 * matched frame by frame. Sixty, not ten: 0 lost of 10 only bounds the loss
 * rate at 26 % (95 % Clopper–Pearson), while 0 of 59 bounds it at 5 % — the
 * M3 pass criterion (docs/validation/range-test.md). One minute per point.
 * Antennas must be fitted on both boards before this runs. */
#define LINK_TEST_FRAMES 60

__attribute__((unused)) static void link_test_tx(void) {
    printf("--- LoRa link test: %d frames, 1/s, %+d dBm ---\n", LINK_TEST_FRAMES,
           CONFIG_SWARM_TX_POWER_DBM);
    if (lora_radio_chip() == LORA_CHIP_UNKNOWN) {
        printf("  skipped: no radio\n");
        return;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for (uint8_t seq = 1; seq <= LINK_TEST_FRAMES; seq++) {
        uint8_t frame[12] = {0xA5, 0x5A, 'L', 'N', 'K', seq, mac[3], mac[4], mac[5], 0, 0, 0};
        int64_t t0 = esp_timer_get_time();
        esp_err_t err = lora_radio_tx(frame, sizeof(frame));
        int64_t airtime_us = esp_timer_get_time() - t0;
        screen(5, "link TX %u/%d %s", seq, LINK_TEST_FRAMES, err == ESP_OK ? "ok" : "ERR");
        printf("  TX seq %u: %s (%lld ms on air)\n", seq, err == ESP_OK ? "ok" : esp_err_to_name(err),
               (long long)(airtime_us / 1000));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("  link test done — compare with the receiver's RX lines\n");
}

void app_main(void) {
    for (int s = CONFIG_BOARD_PROBE_START_DELAY_S; s > 0; s--) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n=== swarm board_probe (%s, radio profile %s) ===\n\n", SWARM_BOARD_NAME,
           lora_radio_profile()->name);
    i2c_master_bus_handle_t bus = open_sensor_bus();
    if (bus != NULL) {
        bring_up_pmu(bus);   // on the classic the PMU shares this bus; on the S3 pmu opens its own
        // The panel sits on this bus; on the S3 its rail (ALDO1) came up in
        // bring_up_pmu(). Row 1 (the PMU verdict) is already in s_screen.
        if (swarm_display_init(bus) == ESP_OK) {
            printf("  OLED found: the probe's verdicts follow on the screen\n");
        }
        screen(0, "PROBE %s", lora_radio_profile()->name);
        scan_i2c(bus);
    } else {
        bring_up_pmu(NULL);
    }
    probe_radio();
    echo_nmea_and_count_pps();
#if CONFIG_BOARD_PROBE_LINK_TEST
    link_test_tx();
    screen(6, "done");
#else
    printf("--- LoRa link test: off in this build (CONFIG_BOARD_PROBE_LINK_TEST=n) — nothing was transmitted ---\n");
    screen(5, "link test off (no TX)");
    screen(6, "done");
#endif
    printf("\n=== done — copy these lines into hardware/bringup-log.md ===\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
