/* SPDX-License-Identifier: MIT
 *
 * M1 bring-up diagnostic (Step 4, ADR 0027/0028). Reports, once, what is
 * actually wired on a classic satellite: PMU identity and rails, I2C scan,
 * radio identity, and NMEA (plus PPS edges on a board that routes them —
 * the classic does not). Not the shipped satellite firmware; a throwaway
 * tool whose output goes verbatim into hardware/bringup-log.md.
 *
 * The AXP2101 is brought up first through components/pmu: the radio and the
 * GNSS module hang from its ALDO2/ALDO3 rails, off until enabled, so a
 * radio probe before that step would always report "no radio".
 */

#include <stdio.h>

#include "board_pins.h"
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
    printf("  battery %u mV (%s%s), vbus %u mV (%s), sys %u mV\n",
           info.batt_mv, info.battery_present ? "cell fitted" : "no cell",
           info.charging ? ", charging" : "", info.vbus_mv,
           info.vbus_present ? "present" : "absent", info.sys_mv);
}

static void scan_i2c(i2c_master_bus_handle_t bus) {
    printf("--- I2C scan (SDA=%d SCL=%d) ---\n", SWARM_PIN_I2C_SDA, SWARM_PIN_I2C_SCL);
    for (uint16_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, I2C_XFER_TIMEOUT_MS) == ESP_OK) {
            const char *known = "";
            if (addr == SWARM_I2C_ADDR_PMU) known = " (PMU AXP2101)";
            else if (addr == SWARM_I2C_ADDR_OLED) known = " (OLED — SSD1306 on the classics, fitted on both units)";
            else if (addr == SWARM_I2C_ADDR_SCD30) known = " (SCD30)";
            else if (addr == SWARM_I2C_ADDR_BME688) known = " (BME688)";
            printf("  0x%02x ACKed%s\n", addr, known);
        }
    }
    printf("  expected on a classic without the kit wired: 0x%02x, 0x%02x; with the kit: + 0x%02x, 0x%02x\n",
           SWARM_I2C_ADDR_PMU, SWARM_I2C_ADDR_OLED, SWARM_I2C_ADDR_SCD30, SWARM_I2C_ADDR_BME688);
}

static void probe_radio(void) {
    printf("--- LoRa radio probe (433.175 MHz profile, RST=%d) ---\n", SWARM_PIN_LORA_RST);
    lora_chip_t chip = LORA_CHIP_UNKNOWN;
    esp_err_t err = lora_radio_init(&chip);
    if (err == ESP_OK) {
        printf("  found: %s\n", chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262");
#if CONFIG_IDF_TARGET_ESP32
        if (chip == LORA_CHIP_SX1278) {
            printf("  (matches the HPDTek HPD14A on the purchase record; RST=%d confirmed)\n", SWARM_PIN_LORA_RST);
        } else {
            printf("  (an SX1262 on a classic contradicts the purchase record — note it in hardware/bringup-log.md)\n");
        }
#endif
    } else {
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
    if (swarm_board_has_pps()) {
        printf("; %u PPS edge(s)\n", (unsigned)s_pps_edges);
    } else {
        printf("\n");
    }
    if (lines_seen == 0) {
        printf("  no NMEA at all — with ALDO3 on this contradicts the schematic: check GPS_RX=%d/GPS_TX=%d\n",
               SWARM_PIN_GPS_RX, SWARM_PIN_GPS_TX);
    }
}

/* Bench / range link test: sixty tagged frames, one per second, at the
 * configured TX power (CONFIG_SWARM_TX_POWER_DBM: +2 dBm lab floor, +8 dBm
 * Madrid field profile — ADR 0031). The receiver prints each as
 * `RX <rssi> <snr> <hex>`; the hex starts with a5 5a 4c 4e 4b ("LNK") then
 * the sequence number and this chip's MAC tail, so the two consoles can be
 * matched frame by frame. Sixty, not ten: 0 lost of 10 only bounds the loss
 * rate at 26 % (95 % Clopper–Pearson), while 0 of 59 bounds it at 5 % — the
 * M3 pass criterion (docs/validation/range-test.md). One minute per point.
 * Antennas must be fitted on both boards before this runs. */
#define LINK_TEST_FRAMES 60

static void link_test_tx(void) {
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
        printf("  TX seq %u: %s (%lld ms on air)\n", seq, err == ESP_OK ? "ok" : esp_err_to_name(err),
               (long long)(airtime_us / 1000));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("  link test done — compare with the receiver's RX lines\n");
}

void app_main(void) {
    printf("\n=== swarm board_probe (%s) ===\n\n", SWARM_BOARD_NAME);
    i2c_master_bus_handle_t bus = open_sensor_bus();
    if (bus != NULL) {
        bring_up_pmu(bus);   // on the classic the PMU shares this bus; on the S3 pmu opens its own
        scan_i2c(bus);
    } else {
        bring_up_pmu(NULL);
    }
    probe_radio();
    echo_nmea_and_count_pps();
    link_test_tx();
    printf("\n=== done — copy these lines into hardware/bringup-log.md ===\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
