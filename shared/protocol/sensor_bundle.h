/* SPDX-License-Identifier: MIT
 *
 * One sensor sample, as the node assembles it. Authoritative definition —
 * do not duplicate.
 *
 * INTERNAL to the T-Beam firmware: handed from the Core 1 acquisition tasks
 * (PMS7003 on UART, SCD30 on I2C 0x61, BME688 on I2C 0x76) to the RAW_SENSOR
 * frame packer, and carried to a camera payload inside payload_link frames.
 * The node's only wire contract is RAW_SENSOR (protocols/specs/messages.md).
 *
 * Sensor ownership (ADR 0027 / ADR 0023): Core 1 supplies PM2.5 + CO2 (and
 * the SCD30's own T/RH). The authoritative T/RH/P — and the gas-resistance
 * channel (ADR 0029) — come from the EXTERNAL BME688 that every node carries,
 * and are overlaid onto the outbound RAW_SENSOR frame.
 */

#ifndef SWARM_PROTOCOL_SENSOR_BUNDLE_H_
#define SWARM_PROTOCOL_SENSOR_BUNDLE_H_

#include <stdint.h>

#define SWARM_SENSOR_BUNDLE_SIZE         16

/* Validity flags for sensor_bundle_t.flags */
#define SWARM_FLAG_PMS7003_VALID         (1U << 0)
#define SWARM_FLAG_SCD30_VALID           (1U << 1)
#define SWARM_FLAG_ENV_VALID             (1U << 2)  /* T/RH/P valid — set after the external BME688 read (ADR 0027); the other acquisition tasks leave it clear */
#define SWARM_FLAG_THERMAL_COMP_APPLIED  (1U << 3)
#define SWARM_FLAG_GAS_VALID             (1U << 7)  /* gas_raw valid; clear during BME688 heater warm-up. Bit position matches RAW_SENSOR's, so the packer's pass-through mask needs no remapping (ADR 0029) */

#pragma pack(push, 1)
typedef struct {
    uint16_t pm25_raw;       /* PM2.5 in µg/m³ × 10 (e.g. 250 = 25.0 µg/m³) — Plantower PMS family (PMS5003/PMSA003 first tests, PMS7003 later; same frame) */
    uint16_t co2_raw;        /* CO2 in ppm (valid range 400-5000) — SCD30 */
    int16_t  temp_raw;       /* Temperature × 100 — SCD30's own sensor (secondary; authoritative T is the external BME688, ADR 0027) */
    uint16_t humid_raw;      /* Relative humidity × 100 — SCD30's own sensor (secondary; see temp_raw) */
    uint16_t pressure_raw;   /* Pressure in hPa × 10, matching RAW_SENSOR. Acquisition leaves 0 — the BME688 read fills P (ADR 0027). Back-fix recorded in backlog ADR 0013. */
    uint8_t  flags;          /* see SWARM_FLAG_* */
    uint8_t  seq_number;     /* 0-255, increments per bundle, wraps */
    uint16_t gas_raw;        /* BME688 gas resistance in Ω / 100 (1 LSB = 100 Ω, span 100 Ω-6.5535 MΩ); 0 = invalid, clamp at both ends (ADR 0029) */
    uint8_t  reserved[1];    /* zeroed; pads to the declared 16-byte size */
    /* CRC-8 over bytes 0..14: poly 0x07, init 0x00, MSB-first, no reflection,
     * no final XOR (CRC-8/SMBUS); check value 0xF4 for the ASCII string
     * "123456789". Reference implementation: crc8() in
     * node_payload/sensor_bundle.py of the node-computer-payload submodule,
     * unit-tested against that check value; any C implementation must match
     * it bit-for-bit. */
    uint8_t  checksum;
} sensor_bundle_t;
#pragma pack(pop)

_Static_assert(sizeof(sensor_bundle_t) == SWARM_SENSOR_BUNDLE_SIZE,
               "sensor_bundle_t must be exactly 16 bytes");

#endif /* SWARM_PROTOCOL_SENSOR_BUNDLE_H_ */
