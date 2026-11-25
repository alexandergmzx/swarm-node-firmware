# Firmware

**One node, two board variants** ([ADR 0027](../docs/adr/0027-prototype-fleet-external-sensor-kit.md)): the classic T-Beam (ESP32) runs the satellite role, the T-Beam S3 Supreme runs the receiver — two separate ESP-IDF apps sharing pin maps, radio driver and wire-format headers ([ADR 0028](../docs/adr/0028-esp-idf-for-the-prototype.md); the why is in [`tbeam/README.md`](tbeam/README.md)). A camera node's payload computer ([ADR 0008](../docs/adr/0008-four-module-node-pi-payload-computer.md) / [0009](../docs/adr/0009-image-thumbnails-over-lora.md)) runs Linux, and its Python code lives in the [`node-computer-payload/`](../node-computer-payload/) git submodule (repo `swarm-node-payload`) — the deliberate exception to the Python-location rule, see [CLAUDE.md](../CLAUDE.md#where-python-lives) — not here.

| Target | Subdirectory | Toolchain |
|--------|--------------|-----------|
| ESP32 + ESP32-S3 (classic T-Beam satellites, T-Beam S3 Supreme receiver) | [`tbeam/`](tbeam/) | ESP-IDF v5.4.1 + `idf.py` |
| Shared headers | [`shared/protocol/`](shared/protocol/) | C headers, no toolchain |

Contracts in [`shared/protocol/`](shared/protocol/):

- [`sensor_bundle.h`](shared/protocol/sensor_bundle.h) — the 16-byte `sensor_bundle_t`, **internal** to the T-Beam firmware: the hand-off from the Core 1 acquisition threads to the frame packer, guarded by its size assertion.
- [`payload_link.h`](shared/protocol/payload_link.h) — Pi ↔ ESP32-S3 framed UART link on camera nodes (spec: [`protocols/specs/payload-uart-link.md`](../protocols/specs/payload-uart-link.md)). A real wire format between two independently-built binaries, and sacred.

The node's only **on-air** contract is `RAW_SENSOR` ([`protocols/specs/messages.md`](../protocols/specs/messages.md)).

Structs must be byte-identical on all sides; include/mirror the headers rather than duplicating them.
