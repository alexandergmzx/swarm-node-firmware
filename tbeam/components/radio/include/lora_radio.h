/* SPDX-License-Identifier: MIT
 *
 * C facade over RadioLib (ADR 0028 decision 5): one API for the two radios
 * in the fleet — the SX1278 on the classic satellites (HPDTek HPD14A module,
 * 433 MHz; ADR 0027 second bring-up note) and the SX1262 on the S3 Supreme
 * Supreme (433 MHz or 915 MHz variant). The chip is probed at boot as a
 * guard, not a discovery.
 *
 * Interop profile (every radio of one mesh must agree — ADR 0016, ADR 0029,
 * ADR 0031), chosen in Kconfig ("Swarm radio profile"):
 *   433-LAB  433.175 MHz / 125 kHz / SF7   26 B frame = 61.7 ms on air
 *   915-MX   916.700 MHz / 500 kHz / SF9   26 B frame = 51.5 ms on air
 * Both: CR 4:5 / explicit header / PHY CRC on / 8-symbol preamble / private
 * sync word 0x12 / TX power CONFIG_SWARM_TX_POWER_DBM (default +2 dBm).
 * RadioLib's SX1262 begin() takes the same 1-byte
 * sync-word convention as SX127x (0x12), translating it internally to the
 * SX126x register encoding (0x1424) — callers of this facade never see that
 * translation.
 */

#ifndef SWARM_LORA_RADIO_H_
#define SWARM_LORA_RADIO_H_

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LORA_CHIP_UNKNOWN = 0,
    LORA_CHIP_SX1278,   /* SX127x family — the classic T-Beam's HPD14A */
    LORA_CHIP_SX1262,
} lora_chip_t;

/** The interop profile this build was configured for. */
typedef struct {
    const char *name;      /* "433-LAB" or "915-MX" */
    uint32_t freq_khz;
    uint16_t bw_khz;
    uint8_t sf;
    uint8_t cr_denom;      /* 5 = CR 4:5 */
    uint8_t sync_word;
    uint16_t preamble;
    int8_t tx_dbm;
} lora_profile_t;

/**
 * Time on air in microseconds of a len-byte packet at this build's profile:
 * CR 4:5, explicit header, PHY CRC on, 8-symbol preamble, no low-data-rate
 * optimisation (symbols are 1.024 ms at both profiles, under its 16 ms
 * threshold). Semtech's formula: (8 + 4.25 + 8 + ceil((8L - 4SF + 44) / 4SF)
 * * 5) symbols. 26 B gives 61 696 us at 433-LAB and 51 456 us at 915-MX.
 * A constant expression, so callers can _Static_assert on it.
 */
#define LORA_RADIO_AIRTIME_US(len)                                                          \
    ((49u + 4u * (8u + 5u * ((8u * (len) + 43u) / (4u * CONFIG_SWARM_RADIO_SF)))) *          \
     ((1000u << CONFIG_SWARM_RADIO_SF) / CONFIG_SWARM_RADIO_BW_KHZ) / 4u)

/**
 * Probe the radio wired per board_pins.h, bring it up at the interop
 * profile, and leave it in standby. On esp32 tries the SX1278 first
 * (RegVersion == 0x12 over SPI) with the SX1262 as a guard; on esp32s3 the
 * other way round — one call covers both boards. Requires swarm_pmu_init()
 * to have enabled the radio rail first (components/pmu).
 *
 * @param out_chip  set to the chip that was found; LORA_CHIP_UNKNOWN on
 *                  failure (leave the caller's diagnostics to report that).
 */
esp_err_t lora_radio_init(lora_chip_t *out_chip);

/** Blocking transmit — the satellite's whole TX budget is one TDMA slot
 * (LORA_RADIO_AIRTIME_US: 61.7 ms or 51.5 ms for a 26 B frame), so blocking
 * here does not risk the slot arithmetic (ADR 0016). Returns once the packet
 * is fully on air. */
esp_err_t lora_radio_tx(const uint8_t *data, size_t len);

/** Put the radio into continuous RX. The receiver app (ADR 0028) never
 * transmits, so this is called once at boot and left running. */
esp_err_t lora_radio_rx_start(void);

/**
 * Block until a packet arrives (or timeout), then read it.
 *
 * @param buf         destination buffer
 * @param buf_len     capacity of buf
 * @param out_len     bytes actually written to buf
 * @param out_rssi    RSSI of the received packet, dBm
 * @param out_snr     SNR of the received packet, dB
 * @param timeout_ms  0 = wait forever
 * @return ESP_OK on a packet, ESP_ERR_TIMEOUT if none arrived in time,
 *         ESP_ERR_INVALID_SIZE if the packet does not fit buf.
 */
esp_err_t lora_radio_rx_poll(uint8_t *buf, size_t buf_len, size_t *out_len,
                              float *out_rssi, float *out_snr,
                              uint32_t timeout_ms);

/** Which chip lora_radio_init() found, once it has run. */
lora_chip_t lora_radio_chip(void);

/** The profile this build transmits and listens on (valid before init). */
const lora_profile_t *lora_radio_profile(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_LORA_RADIO_H_ */
