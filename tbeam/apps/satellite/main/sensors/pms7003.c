// SPDX-License-Identifier: MIT
//
// Frame layout verified against docs/datasheets/plantower/pms7003/
// plantower_pms7003_data-manual_en_v2.5_mirror.pdf, Appendix I: 0x42 0x4D,
// u16BE frame length (=28, i.e. 13 data words + checksum), 13 u16BE data
// words, u16BE checksum = sum of the preceding 30 bytes. Data5 (bytes
// 12-13) is PM2.5 under atmospheric conditions, the value this driver
// reports. The datasheet states response time (<1 s single, <=10 s total)
// but no power-on warm-up figure; the 30 s delay below is the fan/airflow
// settling margin community practice around this sensor uses, not a
// datasheet spec — worth shortening once bench data justifies it.

#include "sensors/pms7003.h"

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "shared_state.h"

static const char *TAG = "pms7003";

#define PMS_UART_NUM UART_NUM_2
#define PMS_UART_BAUD 9600
#define FRAME_LEN 32
#define WARMUP_MS (30 * 1000)

static uint16_t crc_check(const uint8_t *frame) {
    uint16_t sum = 0;
    for (int i = 0; i < 30; i++) sum += frame[i];
    return sum;
}

static void pms7003_task(void *arg) {
    (void)arg;
    int64_t power_on_us = esp_timer_get_time();
    uint8_t frame[FRAME_LEN];
    int filled = 0;
    bool synced = false;

    while (1) {
        uint8_t byte;
        int n = uart_read_bytes(PMS_UART_NUM, &byte, 1, pdMS_TO_TICKS(1000));
        if (n <= 0) continue;

        if (!synced) {
            if (filled == 0 && byte == 0x42) {
                frame[filled++] = byte;
            } else if (filled == 1 && byte == 0x4D) {
                frame[filled++] = byte;
                synced = true;
            } else {
                filled = 0;
            }
            continue;
        }

        frame[filled++] = byte;
        if (filled < FRAME_LEN) continue;

        filled = 0;
        synced = false;  // re-sync from the next 0x42 regardless of outcome

        uint16_t frame_len_field = ((uint16_t)frame[2] << 8) | frame[3];
        if (frame_len_field != 2 * 13 + 2) {
            ESP_LOGD(TAG, "unexpected frame length field %u, dropping frame", frame_len_field);
            continue;
        }
        uint16_t received_crc = ((uint16_t)frame[30] << 8) | frame[31];
        if (crc_check(frame) != received_crc) {
            ESP_LOGD(TAG, "checksum mismatch, dropping frame");
            continue;
        }

        if ((esp_timer_get_time() - power_on_us) < (WARMUP_MS * 1000)) {
            continue;  // discard readings during warm-up; leave PMS7003_VALID clear
        }

        uint16_t pm25_atmospheric = ((uint16_t)frame[12] << 8) | frame[13];
        // Wire/bundle convention is ug/m3 x 10 (sensor_bundle.h); the sensor's
        // own resolution is 1 ug/m3.
        swarm_state_set_pms7003((uint16_t)(pm25_atmospheric * 10));
    }
}

esp_err_t swarm_pms7003_start(void) {
    gpio_config_t set_cfg = {
        .pin_bit_mask = 1ULL << SWARM_PIN_PMS_SET,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&set_cfg);
    gpio_set_level(SWARM_PIN_PMS_SET, 1);  // active (not standby)

    uart_config_t cfg = {
        .baud_rate = PMS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(PMS_UART_NUM, 512, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(PMS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;
    err = uart_set_pin(PMS_UART_NUM, SWARM_PIN_PMS_TX, SWARM_PIN_PMS_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // Core 1, low priority: blocking UART stream, never on Core 0 (ADR 0028).
    BaseType_t ok = xTaskCreatePinnedToCore(pms7003_task, "pms7003", 3072,
                                             NULL, 5, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
