/* SPDX-License-Identifier: MIT
 *
 * Onboard 128x64 OLED as a bench status screen (backlog ADR 0032,
 * docs/validation/oled-demos.md): the classic T-Beam's SSD1306 and the
 * S3 Supreme's SH1106, both at I2C 0x3C (0x3D on some S3 V3.1 units), driven
 * in page-addressing mode, which both controllers support.
 *
 * Text only: a 5x7 font in 6-pixel cells, 21 columns x 8 rows, one 1 KB
 * framebuffer. An app registers one render callback; a task on Core 1 at
 * priority 4 (below every sensor task) calls it and pushes the framebuffer
 * once per period. Nothing here runs on Core 0.
 *
 * CONFIG_SWARM_DISPLAY=n compiles every function to a no-op, so a field
 * build carries no display code and never lights the panel.
 */

#ifndef SWARM_DISPLAY_H_
#define SWARM_DISPLAY_H_

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWARM_DISPLAY_COLS 21
#define SWARM_DISPLAY_ROWS 8

/**
 * Find the panel and initialise it.
 *
 * @param bus  the I2C master bus the panel sits on (the satellite's sensor
 *             bus; on a classic the PMU's shared bus). NULL opens bus 0 on
 *             SWARM_PIN_I2C_SDA/SCL itself, for an app that has no sensor bus
 *             (the S3 receiver).
 * @return ESP_ERR_NOT_FOUND when no panel answers at 0x3C or 0x3D (the
 *         caller carries on without a screen), ESP_OK otherwise.
 */
esp_err_t swarm_display_init(i2c_master_bus_handle_t bus);

/** True once swarm_display_init() found a panel. */
bool swarm_display_present(void);

/** Blank the framebuffer (not the panel: that happens on the next flush). */
void swarm_display_clear(void);

/** Write one text row (0-7), truncated or space-padded to 21 columns.
 * Row 0 is drawn inverted, as a title bar. Only valid inside the render
 * callback, or before swarm_display_start(). */
void swarm_display_row(int row, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/** Light one pixel (x 0-127, y 0-63; y 0 at the top). Draw after the text
 * rows, since swarm_display_row() rewrites its whole 8-pixel band. */
void swarm_display_pixel(int x, int y);

/** A filled vertical bar from y_top down to y_bottom (inclusive), width w. */
void swarm_display_vbar(int x, int w, int y_top, int y_bottom);

/** Push the framebuffer to the panel. */
esp_err_t swarm_display_flush(void);

/**
 * Start the refresh task: every period_ms it clears the framebuffer, calls
 * render() and flushes. Core 1, priority 4.
 */
esp_err_t swarm_display_start(void (*render)(void), uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_DISPLAY_H_ */
