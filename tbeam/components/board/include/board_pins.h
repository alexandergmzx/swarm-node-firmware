/* SPDX-License-Identifier: MIT
 *
 * Per-variant pin and bus map — the machine-readable source of truth the
 * prose tables in hardware/wiring/interconnect-diagram.md mirror (ADR 0028
 * decision 6; it replaces the devicetree overlay the Zephyr plan promised).
 *
 * Classic T-Beam values are CONFIRMED against the vendor V1.2 schematic
 * (docs/datasheets/lilygo/t-beam-classic/, 2026-09-03): both bench units are
 * that one design ("T22_GPS_V1.1 replaced with AXP2101"). What is still
 * pending is the electrical confirmation per unit, recorded in
 * hardware/bringup-log.md when the probes run. S3 Supreme values are
 * confirmed against the vendor V3.0 schematic and hardware page.
 */

#ifndef SWARM_BOARD_PINS_H_
#define SWARM_BOARD_PINS_H_

#include <stdbool.h>
#include <stdint.h>

#if CONFIG_IDF_TARGET_ESP32

/* ---- Classic LilyGO T-Beam (original ESP32) — satellite -------------- */
#define SWARM_BOARD_NAME        "t-beam-classic"

/* LoRa: HPDTek HPD14A module = Semtech SX1278, 433 MHz matching (ADR 0031).
 * RST is IO23 per the schematic (sheet 1, "IO23=IRQ/RESET"); RadioLib's
 * upstream T-Beam example (GPIO 14) was wrong for this board. DIO0 (26) is
 * the SX127x TX/RX-done IRQ; DIO1 (33) and DIO2 (32) are the module's other
 * IRQ lines. There is no BUSY line on this part — SWARM_PIN_LORA_BUSY names
 * DIO2 only so the SX126x guard probe in components/radio compiles for
 * both targets. */
#define SWARM_PIN_LORA_SCK      5
#define SWARM_PIN_LORA_MISO     19
#define SWARM_PIN_LORA_MOSI     27
#define SWARM_PIN_LORA_CS       18
#define SWARM_PIN_LORA_RST      23
#define SWARM_PIN_LORA_DIO0     26   /* SX1278 IRQ (TX done / RX done) */
#define SWARM_PIN_LORA_DIO1     33
#define SWARM_PIN_LORA_BUSY     32   /* DIO2 on the HPD14A; read only by the SX126x guard probe */

/* One I2C bus carries the PMU (0x34), the fitted SSD1306 OLED (0x3C), the
 * SCD30 (0x61) and the BME688 (0x76). */
#define SWARM_PIN_I2C_SDA       21
#define SWARM_PIN_I2C_SCL       22

/* The AXP2101 sits on that same bus (schematic sheet 5): components/pmu
 * attaches to the shared master bus instead of opening a second one. It
 * feeds the radio from ALDO2 (LORA_VCC) and the GNSS module from ALDO3
 * (GPS_VDD). Measured on classic-A after a cold start (2026-09-03): ALDO2 is
 * OFF with a 2.8 V setting, ALDO3 is ON at 3.3 V — so components/pmu's
 * enable is a hard requirement for the radio and a no-op for the GNSS. */
#define SWARM_PIN_PMU_SDA       21
#define SWARM_PIN_PMU_SCL       22
#define SWARM_PIN_PMU_IRQ       35

/* GPS: u-blox NEO-6M (bench-identified 2026-09-03, both units), 9600 8N1.
 * No PPS on this board: the module's TIME_PULSE feeds only the red fix LED
 * through R34 (schematic sheet 3), so gps_time disciplines the slot clock
 * from NMEA arrival by design (ADR 0027 item 6). A bodge wire from the
 * module side of R34 to GPIO 14 (free) is the one way to change that. */
#define SWARM_PIN_GPS_RX        34   /* ESP RX <- GPS TX (input-only pin: legal) */
#define SWARM_PIN_GPS_TX        12
#define SWARM_PIN_GPS_PPS       (-1) /* not routed on this board */

/* PMS7003 on UART2 via the GPIO matrix, plus its standby line. GPIO 14 is
 * deliberately left free (the PPS bodge candidate above). */
#define SWARM_PIN_PMS_RX        36   /* ESP RX <- PMS TX (input-only: legal) */
#define SWARM_PIN_PMS_TX        13
#define SWARM_PIN_PMS_SET       15
#define SWARM_PIN_BOOST_EN      25   /* 5 V boost enable, if the module exposes one */

#elif CONFIG_IDF_TARGET_ESP32S3

/* ---- LilyGO T-Beam S3 Supreme V3.0 — receiver (vendor schematic + hardware page; ADR 0023 item 2) */
#define SWARM_BOARD_NAME        "t-beam-s3-supreme"

#define SWARM_PIN_LORA_SCK      12
#define SWARM_PIN_LORA_MISO     13
#define SWARM_PIN_LORA_MOSI     11
#define SWARM_PIN_LORA_CS       10
#define SWARM_PIN_LORA_RST      5
#define SWARM_PIN_LORA_DIO0     (-1) /* SX1262: no DIO0 */
#define SWARM_PIN_LORA_DIO1     1
#define SWARM_PIN_LORA_BUSY     4

/* I2C bus 0 (sensors): onboard BME280 at 0x77 and SH1106 at 0x3C, so the
 * external BME688 is strapped to 0x76 fleet-wide (ADR 0027 decision 2). */
#define SWARM_PIN_I2C_SDA       17
#define SWARM_PIN_I2C_SCL       18

/* I2C bus 1 (power): AXP2101 at 0x34 and the PCF8563 RTC at 0x51 — a
 * separate bus on this board (vendor hardware page). components/pmu opens
 * it itself. The radio hangs from ALDO3, the GNSS module from ALDO4. */
#define SWARM_PIN_PMU_SDA       42
#define SWARM_PIN_PMU_SCL       41
#define SWARM_PIN_PMU_IRQ       40

#define SWARM_PIN_GPS_RX        9
#define SWARM_PIN_GPS_TX        8
#define SWARM_PIN_GPS_PPS       6    /* L76K 1PPS — do not reuse (ADR 0023) */

#define SWARM_PIN_PMS_RX        39
#define SWARM_PIN_PMS_TX        38
#define SWARM_PIN_PMS_SET       2
#define SWARM_PIN_BOOST_EN      (-1)

#else
#error "Unsupported target: this project builds for esp32 (classic T-Beam) or esp32s3 (S3 Supreme)"
#endif

/* Bus addresses are identical on both variants — one firmware constant. */
#define SWARM_I2C_ADDR_PMU      0x34
#define SWARM_I2C_ADDR_OLED     0x3C   /* SSD1306 on the classics, SH1106 on the S3 */
#define SWARM_I2C_ADDR_SCD30    0x61
#define SWARM_I2C_ADDR_BME688   0x76

/** True when the PMU shares the sensor I2C bus (classic) rather than having its own (S3). */
#define SWARM_PMU_SHARES_SENSOR_BUS \
    (SWARM_PIN_PMU_SDA == SWARM_PIN_I2C_SDA && SWARM_PIN_PMU_SCL == SWARM_PIN_I2C_SCL)

/** True when this board routes GPS 1PPS to a pin the MCU can read. */
static inline bool swarm_board_has_pps(void)
{
    return SWARM_PIN_GPS_PPS >= 0;
}

#endif /* SWARM_BOARD_PINS_H_ */
