// SPDX-License-Identifier: MIT
//
// Forced-mode configuration (filter off, os_hum 16x/os_pres 1x/os_temp 2x,
// heater 300 C / 100 ms) follows the vendored driver's own
// examples/forced_mode/forced_mode.c. Gas validity is BME68X_GASM_VALID_MSK
// AND BME68X_HEAT_STAB_MSK together — Bosch's own recommendation: a gas
// reading only means something once the heater has both finished its
// measurement cycle and stabilized at the target temperature.

#include "sensors/bme688.h"

#include <math.h>
#include <string.h>

#include "bme68x.h"
#include "board_pins.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "sdkconfig.h"
#include "sensors/scd30.h"
#include "shared_state.h"

static const char *TAG = "bme688";

#define BME688_XFER_TIMEOUT_MS 1000
#define BME688_HEATER_TEMP_C 300
#define BME688_HEATER_DUR_MS 100
#define BME688_CYCLE_MS 5000  // one reading per TDMA superframe is plenty (ADR 0016 cadence)
#define SCD30_PRESSURE_MIN_MBAR 700
#define SCD30_PRESSURE_MAX_MBAR 1400

static i2c_master_dev_handle_t s_dev = NULL;

static BME68X_INTF_RET_TYPE i2c_read(uint8_t reg_addr, uint8_t *reg_data,
                                      uint32_t length, void *intf_ptr) {
    (void)intf_ptr;
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg_addr, 1, reg_data,
                                                 length, BME688_XFER_TIMEOUT_MS);
    return err == ESP_OK ? 0 : -1;
}

static BME68X_INTF_RET_TYPE i2c_write(uint8_t reg_addr, const uint8_t *reg_data,
                                       uint32_t length, void *intf_ptr) {
    (void)intf_ptr;
    uint8_t buf[32];
    if (length + 1 > sizeof(buf)) return -1;
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, length);
    esp_err_t err = i2c_master_transmit(s_dev, buf, length + 1, BME688_XFER_TIMEOUT_MS);
    return err == ESP_OK ? 0 : -1;
}

static void bme688_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    } else {
        esp_rom_delay_us(period);
    }
}

static void bme688_task(void *arg) {
    (void)arg;

    struct bme68x_dev dev = {
        .intf = BME68X_I2C_INTF,
        .read = i2c_read,
        .write = i2c_write,
        .delay_us = bme688_delay_us,
        .amb_temp = 25,  // initial guess for the internal T compensation; irrelevant after warm-up
    };
    if (bme68x_init(&dev) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_init failed — no BME688 answered at 0x%02x", SWARM_I2C_ADDR_BME688);
        vTaskDelete(NULL);
        return;
    }

    struct bme68x_conf conf = {
        .filter = BME68X_FILTER_OFF,
        .odr = BME68X_ODR_NONE,
        .os_hum = BME68X_OS_16X,
        .os_pres = BME68X_OS_1X,
        .os_temp = BME68X_OS_2X,
    };
    if (bme68x_set_conf(&conf, &dev) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_conf failed");
    }

    struct bme68x_heatr_conf heatr_conf = {
        .enable = BME68X_ENABLE,
        .heatr_temp = BME688_HEATER_TEMP_C,
        .heatr_dur = BME688_HEATER_DUR_MS,
    };
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &dev) != BME68X_OK) {
        ESP_LOGE(TAG, "bme68x_set_heatr_conf failed");
    }

    while (1) {
        if (bme68x_set_op_mode(BME68X_FORCED_MODE, &dev) != BME68X_OK) {
            ESP_LOGW(TAG, "set_op_mode failed, retrying next cycle");
            vTaskDelay(pdMS_TO_TICKS(BME688_CYCLE_MS));
            continue;
        }

        uint32_t del_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &dev) +
                           (uint32_t)heatr_conf.heatr_dur * 1000;
        dev.delay_us(del_us, dev.intf_ptr);

        struct bme68x_data data;
        uint8_t n_fields = 0;
        int8_t rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);
        if (rslt == BME68X_OK && n_fields > 0) {
            bool gas_valid = (data.status & (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)) ==
                              (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK);
            float pressure_hpa = data.pressure / 100.0f;
            uint16_t pressure_raw = (uint16_t)lrintf(pressure_hpa * 10.0f);
            uint16_t gas_raw = gas_valid
                                    ? (uint16_t)fminf(65535.0f, fmaxf(1.0f, lrintf(data.gas_resistance / 100.0f)))
                                    : 0;

            swarm_state_set_bme688(data.temperature, data.humidity, pressure_raw,
                                    gas_raw, gas_valid);

            uint16_t pressure_mbar = (uint16_t)lrintf(pressure_hpa);
            if (pressure_mbar < SCD30_PRESSURE_MIN_MBAR || pressure_mbar > SCD30_PRESSURE_MAX_MBAR) {
                pressure_mbar = 0;  // out of the SCD30's compensable range — disable compensation
            }
            swarm_scd30_set_ambient_pressure(pressure_mbar);
        } else {
            ESP_LOGD(TAG, "no new BME688 data this cycle (rslt=%d, n_fields=%u)", rslt, n_fields);
        }

        vTaskDelay(pdMS_TO_TICKS(BME688_CYCLE_MS));
    }
}

esp_err_t swarm_bme688_start(void) {
    esp_err_t err = swarm_i2c_bus_init();
    if (err != ESP_OK) return err;

    // Same rule as the SCD30: probe before attaching, one line if absent.
    if (i2c_master_probe(swarm_i2c_bus(), SWARM_I2C_ADDR_BME688, 200) != ESP_OK) {
        ESP_LOGW(TAG, "no BME688 at 0x%02x — task not started, ENV/GAS_VALID stay clear",
                 SWARM_I2C_ADDR_BME688);
        return ESP_OK;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SWARM_I2C_ADDR_BME688,
        .scl_speed_hz = CONFIG_SWARM_I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(swarm_i2c_bus(), &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;

    // Core 1, low priority: blocking I2C + heater dwell, never on Core 0.
    BaseType_t ok = xTaskCreatePinnedToCore(bme688_task, "bme688", 3584, NULL, 5, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
