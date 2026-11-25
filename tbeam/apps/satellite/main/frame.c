// SPDX-License-Identifier: MIT

#include "frame.h"

#include <math.h>

// RAW_SENSOR carries no CRC of its own — the LoRa PHY CRC covers the whole
// frame (protocols/specs/messages.md) — so this is pure byte layout, no
// checksum. Little-endian stores are written byte-by-byte on purpose: this
// is the one place a silent endianness or alignment assumption would break
// the wire contract, so it is spelled out rather than relied on.

static void put_u16le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void put_i16le(uint8_t *p, int16_t v) { put_u16le(p, (uint16_t)v); }

static void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_i32le(uint8_t *p, int32_t v) { put_u32le(p, (uint32_t)v); }

// Flags bits 0,1,2,3,7 pass through from the bundle (PMS7003_VALID,
// SCD30_VALID, ENV_VALID, THERMAL_COMP_APPLIED, GAS_VALID); bits 4,5 are set
// here from the caller's GPS/time state, not from the bundle; bit 6 stays 0
// (reserved for ANOMALY, backlog 0019).
#define BUNDLE_PASSTHROUGH_MASK 0x8F

void swarm_frame_pack_raw_sensor(uint8_t node_id, uint32_t timestamp,
                                  int32_t lat_1e7, int32_t lon_1e7,
                                  bool gps_fix_valid, bool time_valid,
                                  const swarm_env_overlay_t *env,
                                  const sensor_bundle_t *bundle,
                                  uint8_t out[SWARM_RAW_SENSOR_SIZE]) {
    out[0] = (uint8_t)((0x0 << 4) | (node_id & 0x0F));
    out[1] = bundle->seq_number;
    put_u32le(&out[2], timestamp);
    put_i32le(&out[6], gps_fix_valid ? lat_1e7 : 0);
    put_i32le(&out[10], gps_fix_valid ? lon_1e7 : 0);
    put_i16le(&out[14], (int16_t)lrintf(env->temp_c * 100.0f));

    float humid_steps = env->humidity_pct * 2.0f;
    if (humid_steps < 0.0f) humid_steps = 0.0f;
    if (humid_steps > 200.0f) humid_steps = 200.0f;
    out[16] = (uint8_t)lrintf(humid_steps);

    put_u16le(&out[17], bundle->pressure_raw);  // pass-through; BME688 overlay fills it, 0 = invalid
    put_u16le(&out[19], bundle->pm25_raw);
    put_u16le(&out[21], bundle->co2_raw);
    put_u16le(&out[23], bundle->gas_raw);  // pass-through; BME688 overlay fills it, 0 = invalid

    uint8_t flags = (uint8_t)(bundle->flags & BUNDLE_PASSTHROUGH_MASK);
    if (gps_fix_valid) flags |= 0x10;  // bit 4
    if (time_valid) flags |= 0x20;     // bit 5
    out[25] = flags;
}
