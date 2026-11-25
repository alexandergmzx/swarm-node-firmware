// SPDX-License-Identifier: MIT
//
// Protocol verified against docs/app-notes/sensirion/
// sensirion_scd30_interface-description_v1.0.pdf (Sensirion, v1.0, May 2020;
// filed in the parent repository's reference library since 2026-07-28), sections 1.1.2,
// 1.1.3, 1.4. CRC-8 (poly 0x31, init 0xFF, no reflection,
// no final XOR) checked this session against both the datasheet's own
// worked example (CRC(0xBEEF) == 0x92) and the CRC-8/NRSC-5 catalog check
// value (CRC("123456789") == 0xF7).
//
// SCD30 does not support I2C repeated start (sec 1.1) — every command that
// both writes and reads is two separate transactions with a >3 ms gap, per
// sec 1.4.4/1.4.5, never i2c_master_transmit_receive().

#include "sensors/scd30.h"

#include <math.h>
#include <string.h>

#include "board_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "sdkconfig.h"
#include "shared_state.h"

static const char *TAG = "scd30";

#define SCD30_MEASUREMENT_INTERVAL_S 2  // ADR 0016 cadence
#define SCD30_XFER_TIMEOUT_MS 1000
#define SCD30_POST_WRITE_DELAY_MS 5     // > 3 ms per the interface description
#define SCD30_CLOCK_STRETCH_TIMEOUT_US (200 * 1000)  // covers the 150 ms/day worst case (sec 1.1)
#define SCD30_CO2_MIN_PPM 400
#define SCD30_CO2_MAX_PPM 5000

static i2c_master_dev_handle_t s_dev = NULL;

static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static esp_err_t write_cmd(uint16_t cmd) {
    uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), SCD30_XFER_TIMEOUT_MS);
}

static esp_err_t write_cmd_arg(uint16_t cmd, uint16_t arg) {
    uint8_t buf[5];
    buf[0] = (uint8_t)(cmd >> 8);
    buf[1] = (uint8_t)(cmd & 0xFF);
    buf[2] = (uint8_t)(arg >> 8);
    buf[3] = (uint8_t)(arg & 0xFF);
    buf[4] = crc8(&buf[2], 2);
    return i2c_master_transmit(s_dev, buf, sizeof(buf), SCD30_XFER_TIMEOUT_MS);
}

// Reads `num_words` (data+CRC) triplets, verifying each CRC, into `words`.
static esp_err_t read_words(uint16_t cmd, uint16_t *words, size_t num_words) {
    esp_err_t err = write_cmd(cmd);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(SCD30_POST_WRITE_DELAY_MS));

    uint8_t raw[6 * 3];  // largest read here is read-measurement, 6 words
    size_t n = num_words * 3;
    err = i2c_master_receive(s_dev, raw, n, SCD30_XFER_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    for (size_t i = 0; i < num_words; i++) {
        const uint8_t *w = &raw[i * 3];
        if (crc8(w, 2) != w[2]) {
            ESP_LOGW(TAG, "CRC mismatch on word %u of command 0x%04x", (unsigned)i, cmd);
            return ESP_ERR_INVALID_CRC;
        }
        words[i] = ((uint16_t)w[0] << 8) | w[1];
    }
    return ESP_OK;
}

static esp_err_t trigger_measurement(uint16_t pressure_mbar) {
    return write_cmd_arg(0x0010, pressure_mbar);
}

esp_err_t swarm_scd30_set_ambient_pressure(uint16_t pressure_mbar) {
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;
    // "For setting a new ambient pressure when continuous measurement is
    // running the whole command has to be written to SCD30" (sec 1.4.1).
    return trigger_measurement(pressure_mbar);
}

static bool data_ready(void) {
    uint16_t status;
    if (read_words(0x0202, &status, 1) != ESP_OK) return false;
    return status == 1;
}

static bool read_measurement(float *co2_ppm, float *temp_c, float *humidity_pct) {
    uint16_t words[6];
    if (read_words(0x0300, words, 6) != ESP_OK) return false;
    // Big-endian IEEE-754: word0<<16 | word1 is the raw bit pattern (sec 1.5).
    uint32_t co2_bits = ((uint32_t)words[0] << 16) | words[1];
    uint32_t temp_bits = ((uint32_t)words[2] << 16) | words[3];
    uint32_t rh_bits = ((uint32_t)words[4] << 16) | words[5];
    memcpy(co2_ppm, &co2_bits, sizeof(*co2_ppm));
    memcpy(temp_c, &temp_bits, sizeof(*temp_c));
    memcpy(humidity_pct, &rh_bits, sizeof(*humidity_pct));
    return true;
}

static void scd30_task(void *arg) {
    (void)arg;

    if (trigger_measurement(0) != ESP_OK) {  // 0 = no pressure compensation yet at boot
        ESP_LOGE(TAG, "failed to start continuous measurement");
    }
    vTaskDelay(pdMS_TO_TICKS(SCD30_POST_WRITE_DELAY_MS));
    if (write_cmd_arg(0x4600, SCD30_MEASUREMENT_INTERVAL_S) != ESP_OK) {
        ESP_LOGE(TAG, "failed to set measurement interval");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SCD30_MEASUREMENT_INTERVAL_S * 1000));

        int retries = 3;
        while (retries-- > 0 && !data_ready()) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (retries < 0) continue;  // matches the loop's own decrement past 0

        float co2_ppm, temp_c, humidity_pct;
        if (!read_measurement(&co2_ppm, &temp_c, &humidity_pct)) continue;

        if (co2_ppm < SCD30_CO2_MIN_PPM || co2_ppm > SCD30_CO2_MAX_PPM) {
            ESP_LOGD(TAG, "CO2 %.0f ppm outside valid range, dropping sample", co2_ppm);
            continue;
        }
        swarm_state_set_scd30((uint16_t)lrintf(co2_ppm),
                               (int16_t)lrintf(temp_c * 100.0f),
                               (uint16_t)lrintf(humidity_pct * 100.0f));
    }
}

esp_err_t swarm_scd30_start(void) {
    esp_err_t err = swarm_i2c_bus_init();
    if (err != ESP_OK) return err;

    // Absence is normal this phase (the env_sen[n] kits arrive in October):
    // probe first, so a missing sensor costs one log line instead of an
    // ESP-IDF "I2C hardware NACK detected" error on every transaction,
    // forever, on the same console as the TX echoes (found 2026-09-03).
    if (i2c_master_probe(swarm_i2c_bus(), SWARM_I2C_ADDR_SCD30, 200) != ESP_OK) {
        ESP_LOGW(TAG, "no SCD30 at 0x%02x — task not started, SCD30_VALID stays clear",
                 SWARM_I2C_ADDR_SCD30);
        return ESP_OK;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SWARM_I2C_ADDR_SCD30,
        .scl_speed_hz = CONFIG_SWARM_I2C_FREQ_HZ,
        .scl_wait_us = SCD30_CLOCK_STRETCH_TIMEOUT_US,
    };
    err = i2c_master_bus_add_device(swarm_i2c_bus(), &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;

    // Core 1, low priority: blocking 2 s measurement cycle, never on Core 0.
    BaseType_t ok = xTaskCreatePinnedToCore(scd30_task, "scd30", 3072, NULL, 5, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
