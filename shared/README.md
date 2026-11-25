# Shared firmware sources

Headers and tiny utility sources that must stay byte-identical across every side that touches them. Keep this directory dependency-free (plain C99, no RTOS or HAL types) — that rule is why these headers ported to ESP-IDF unchanged when the firmware platform moved ([ADR 0028](../../docs/adr/0028-esp-idf-for-the-prototype.md)).

- `protocol/sensor_bundle.h` — the 16-byte `sensor_bundle_t`, **internal** to the T-Beam firmware: the hand-off from its Core 1 acquisition threads to its frame packer, and what a camera payload receives inside `payload_link` frames. It is the single definition of what a sample *is* — `RAW_SENSOR` passes its fields through — so it lives here rather than in the firmware tree. Its comment header also pins the CRC-8 variant (poly 0x07, init 0x00, MSB-first, check 0xF4).
- `protocol/payload_link.h` — Pi ↔ ESP32-S3 framed UART link on camera nodes: a real wire format between two independently-built binaries.

LoRa packet structures belong in `protocol/lora_*.h` next to this file — those are the node's only on-air contracts.
