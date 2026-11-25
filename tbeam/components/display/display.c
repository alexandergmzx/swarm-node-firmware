// SPDX-License-Identifier: MIT
//
// Page-addressed driver for the T-Beam OLEDs. See display.h for the contract.
// The SSD1306 (classic) and the SH1106 (S3 Supreme) share the page-addressing
// commands (0xB0 + page, then the column address as two nibbles), so one
// flush routine serves both; they differ in charge-pump setup and in the
// SH1106's 132-column RAM, whose visible 128 columns start at column 2.

#include "display.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_SWARM_DISPLAY

#include "font5x7.h"

static const char *TAG = "display";

#define PANEL_W 128
#define PANEL_PAGES 8
#define CELL_W 6
#define I2C_TIMEOUT_MS 50
#define PANEL_SCL_HZ 400000

#if CONFIG_IDF_TARGET_ESP32
#define PANEL_NAME "SSD1306"
#define PANEL_COL_OFFSET 0
static const uint8_t k_init[] = {
    0xAE,        // display off
    0xD5, 0x80,  // clock divide / oscillator
    0xA8, 0x3F,  // multiplex 64
    0xD3, 0x00,  // display offset 0
    0x40,        // start line 0
    0x8D, 0x14,  // charge pump on
    0x20, 0x02,  // page addressing mode
    0xA1,        // segment remap (column 127 = SEG0)
    0xC8,        // COM scan descending
    0xDA, 0x12,  // COM pins: alternative, 128x64
    0x81, 0xCF,  // contrast
    0xD9, 0xF1,  // pre-charge
    0xDB, 0x40,  // VCOMH deselect
    0xA4,        // display follows RAM
    0xA6,        // normal, not inverted
    0xAF,        // display on
};
#else
#define PANEL_NAME "SH1106"
#define PANEL_COL_OFFSET 2
static const uint8_t k_init[] = {
    0xAE,        // display off
    0xD5, 0x80,  // clock divide / oscillator
    0xA8, 0x3F,  // multiplex 64
    0xD3, 0x00,  // display offset 0
    0x40,        // start line 0
    0xAD, 0x8B,  // DC-DC converter on
    0xA1,        // segment remap
    0xC8,        // COM scan descending
    0xDA, 0x12,  // COM pins
    0x81, 0x80,  // contrast
    0xD9, 0x22,  // pre-charge
    0xDB, 0x35,  // VCOM deselect
    0x32,        // pump voltage 8.0 V
    0xA4,        // display follows RAM
    0xA6,        // normal, not inverted
    0xAF,        // display on
};
#endif

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_fb[PANEL_PAGES][PANEL_W];
static void (*s_render)(void) = NULL;
static uint32_t s_period_ms = 1000;

static esp_err_t send_cmds(const uint8_t *cmds, size_t n) {
    uint8_t buf[1 + 32];
    while (n > 0) {
        size_t chunk = n > 32 ? 32 : n;
        buf[0] = 0x00;  // control byte: command stream
        memcpy(&buf[1], cmds, chunk);
        esp_err_t err = i2c_master_transmit(s_dev, buf, chunk + 1, I2C_TIMEOUT_MS);
        if (err != ESP_OK) return err;
        cmds += chunk;
        n -= chunk;
    }
    return ESP_OK;
}

static i2c_master_bus_handle_t open_own_bus(void) {
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,
        .sda_io_num = SWARM_PIN_I2C_SDA,
        .scl_io_num = SWARM_PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&cfg, &bus) != ESP_OK) return NULL;
    return bus;
}

esp_err_t swarm_display_init(i2c_master_bus_handle_t bus) {
    if (s_dev != NULL) return ESP_OK;
    if (bus == NULL) {
        bus = open_own_bus();
        if (bus == NULL) {
            ESP_LOGW(TAG, "could not open I2C bus SDA=%d SCL=%d — no screen", SWARM_PIN_I2C_SDA,
                     SWARM_PIN_I2C_SCL);
            return ESP_ERR_NOT_FOUND;
        }
    }
    static const uint16_t addrs[] = {SWARM_I2C_ADDR_OLED, 0x3D};
    uint16_t found = 0;
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        if (i2c_master_probe(bus, addrs[i], I2C_TIMEOUT_MS) == ESP_OK) {
            found = addrs[i];
            break;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "no OLED at 0x%02x or 0x3d — running without a screen", SWARM_I2C_ADDR_OLED);
        return ESP_ERR_NOT_FOUND;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found,
        .scl_speed_hz = PANEL_SCL_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) return err;
    err = send_cmds(k_init, sizeof(k_init));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s init failed: %s", PANEL_NAME, esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    swarm_display_clear();
    swarm_display_flush();
    ESP_LOGI(TAG, "%s at 0x%02x, 21 x 8 text", PANEL_NAME, found);
    return ESP_OK;
}

bool swarm_display_present(void) { return s_dev != NULL; }

void swarm_display_clear(void) { memset(s_fb, 0, sizeof(s_fb)); }

void swarm_display_row(int row, const char *fmt, ...) {
    if (row < 0 || row >= SWARM_DISPLAY_ROWS) return;
    char text[SWARM_DISPLAY_COLS + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    uint8_t *page = s_fb[row];
    memset(page, 0, PANEL_W);
    for (int c = 0; c < SWARM_DISPLAY_COLS && text[c] != '\0'; c++) {
        unsigned char ch = (unsigned char)text[c];
        if (ch < 0x20 || ch > 0x7E) ch = '?';
        memcpy(&page[c * CELL_W], swarm_font5x7[ch - 0x20], 5);
    }
    if (row == 0) {
        for (int x = 0; x < PANEL_W; x++) page[x] = (uint8_t)~page[x];  // title bar
    }
}

esp_err_t swarm_display_flush(void) {
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t data[1 + PANEL_W];
    for (int p = 0; p < PANEL_PAGES; p++) {
        const uint8_t addr[] = {(uint8_t)(0xB0 + p), (uint8_t)(0x00 | (PANEL_COL_OFFSET & 0x0F)),
                                (uint8_t)(0x10 | (PANEL_COL_OFFSET >> 4))};
        esp_err_t err = send_cmds(addr, sizeof(addr));
        if (err != ESP_OK) return err;
        data[0] = 0x40;  // control byte: data stream
        memcpy(&data[1], s_fb[p], PANEL_W);
        err = i2c_master_transmit(s_dev, data, sizeof(data), I2C_TIMEOUT_MS);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static void display_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    uint32_t failures = 0;
    while (1) {
        swarm_display_clear();
        if (s_render) s_render();
        if (swarm_display_flush() != ESP_OK && (failures++ % 60) == 0) {
            ESP_LOGW(TAG, "flush failed (%lu so far)", (unsigned long)failures);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(s_period_ms));
    }
}

esp_err_t swarm_display_start(void (*render)(void), uint32_t period_ms) {
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;
    s_render = render;
    s_period_ms = period_ms ? period_ms : 1000;
    // Core 1, priority 4: below the sensor tasks (5) and far from Core 0's
    // TDMA/GPS work (ADR 0028 tenets 1 and 2).
    BaseType_t ok = xTaskCreatePinnedToCore(display_task, "display", 3072, NULL, 4, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

#else  // !CONFIG_SWARM_DISPLAY: the field build links no display code

esp_err_t swarm_display_init(i2c_master_bus_handle_t bus) {
    (void)bus;
    return ESP_ERR_NOT_SUPPORTED;
}
bool swarm_display_present(void) { return false; }
void swarm_display_clear(void) {}
void swarm_display_row(int row, const char *fmt, ...) {
    (void)row;
    (void)fmt;
}
esp_err_t swarm_display_flush(void) { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t swarm_display_start(void (*render)(void), uint32_t period_ms) {
    (void)render;
    (void)period_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
