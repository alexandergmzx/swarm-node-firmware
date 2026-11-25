// SPDX-License-Identifier: MIT

#include "i2c_bus.h"

#include "board_pins.h"
#include "sdkconfig.h"

static i2c_master_bus_handle_t s_bus = NULL;

esp_err_t swarm_i2c_bus_init(void) {
    if (s_bus != NULL) return ESP_OK;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = SWARM_PIN_I2C_SDA,
        .scl_io_num = SWARM_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

i2c_master_bus_handle_t swarm_i2c_bus(void) { return s_bus; }
