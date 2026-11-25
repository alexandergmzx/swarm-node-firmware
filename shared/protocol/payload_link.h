/* SPDX-License-Identifier: MIT
 *
 * Payload UART link between the Raspberry Pi Zero 2W (payload computer)
 * and the T-Beam S3 (ESP32-S3 radio modem). Authoritative definition —
 * the ESP32-S3 firmware includes this header; the Pi-side Python in
 * The camera payload's node_payload/link.py mirrors these constants. Do not duplicate
 * in C. Design: protocols/specs/payload-uart-link.md (ADR 0008).
 *
 * Framing on the wire: COBS-encoded, 0x00-delimited. Inside each frame:
 * payload_link_hdr_t, payload bytes, then CRC-16/CCITT-FALSE (big-endian)
 * over header + payload.
 */

#ifndef SWARM_PROTOCOL_PAYLOAD_LINK_H_
#define SWARM_PROTOCOL_PAYLOAD_LINK_H_

#include <stdint.h>

#define SWARM_PAYLOAD_LINK_VERSION       1
#define SWARM_PAYLOAD_LINK_BAUD          115200
#define SWARM_PAYLOAD_LINK_HDR_SIZE      5
#define SWARM_PAYLOAD_LINK_CRC_SIZE      2
#define SWARM_PAYLOAD_LINK_MAX_PAYLOAD   256  /* fits one IMAGE_CHUNK handoff */

/* Message IDs. Directions per protocols/specs/payload-uart-link.md. */
typedef enum {
    /* ESP32-S3 → Pi */
    SWARM_PL_MSG_SENSOR_BUNDLE  = 0x01, /* payload: sensor_bundle_t (16 B)   */
    SWARM_PL_MSG_GPS_TIME_FIX   = 0x02, /* UTC + position + PPS timestamp    */
    SWARM_PL_MSG_MESH_EVENT     = 0x03, /* neighbour / ACK notification      */
    /* Pi → ESP32-S3 */
    SWARM_PL_MSG_IMAGE_META     = 0x10, /* announce thumbnail for LoRa TX    */
    SWARM_PL_MSG_IMAGE_CHUNK    = 0x11, /* opaque chunk bytes for LoRa TX    */
    SWARM_PL_MSG_SHUTDOWN_READY = 0x12, /* fs synced; rail may be cut        */
    /* Both directions */
    SWARM_PL_MSG_LINK_ACK       = 0x20, /* payload: acked seq (1 B)          */
} payload_msg_id_t;

#pragma pack(push, 1)
typedef struct {
    uint8_t  version;      /* SWARM_PAYLOAD_LINK_VERSION; mismatch → reject */
    uint8_t  msg_id;       /* payload_msg_id_t */
    uint8_t  seq;          /* 0-255, per-sender, wraps; echoed by LINK_ACK  */
    uint16_t payload_len;  /* little-endian, ≤ SWARM_PAYLOAD_LINK_MAX_PAYLOAD */
} payload_link_hdr_t;
#pragma pack(pop)

_Static_assert(sizeof(payload_link_hdr_t) == SWARM_PAYLOAD_LINK_HDR_SIZE,
               "payload_link_hdr_t must be exactly 5 bytes on the wire");

#endif /* SWARM_PROTOCOL_PAYLOAD_LINK_H_ */
