# swarm-node-firmware

ESP-IDF firmware for low-cost environmental sensor nodes that talk over LoRa in a GPS-timed TDMA
schedule. Each satellite node reads PM2.5, CO2, temperature, humidity, pressure and a gas-resistance
channel. It packs them into a 26-byte `RAW_SENSOR` frame and transmits it once per 5 s superframe
in its own slot. A USB-tethered receiver prints every frame for a ground station.

This firmware is one part of the *swarm* research project (a network of sensor nodes along an
industrial pollution corridor). The project's design record lives in a separate repository, which
is private.

## Boards and sensors

| Role | Board | MCU / target | Radio | GNSS |
|---|---|---|---|---|
| Satellite | LilyGO T-Beam (classic, V1.1/V1.2 with AXP2101) | ESP32, `esp32` | SX1278 (HPD14A), 433 MHz | NEO-6M, no PPS routed |
| Satellite or receiver | LilyGO T-Beam S3 Supreme V3.0 | ESP32-S3, `esp32s3` | SX1262 | L76K, PPS on IO6 |

The external sensor kit on every satellite is a Plantower PMS5003, PMSA003 or PMS7003 (UART, same
32-byte frame), a Sensirion SCD30 (I2C `0x61`) and a Bosch BME688 (I2C `0x76`). Absent sensors are
detected at boot. Their fields are sent with the VALID flag cleared.

## Layout

| Path | What it holds |
|---|---|
| `tbeam/apps/satellite/` | TDMA transmitter. Core 0 runs GPS time discipline and radio TX; Core 1 runs the sensor acquisition tasks |
| `tbeam/apps/receiver/` | Continuous-RX receiver. It prints `RX <rssi> <snr> <hex>` and `FIX …` lines on USB |
| `tbeam/apps/board_probe/` | One-shot bring-up diagnostic: PMU, I2C scan, radio identity, NMEA/PPS, a 60-frame link test |
| `tbeam/components/` | `board` (pin maps), `radio` (RadioLib facade and the interop profile), `pmu` (AXP2101 rails), `gps_time`, `bme68x`, `swarm_proto` |
| `tbeam/third_party/` | RadioLib, XPowersLib and the Bosch BME68x SensorAPI as git submodules, pinned |
| `shared/protocol/` | C headers that must stay byte-identical across builds: `sensor_bundle.h` (internal hand-off) and `payload_link.h` (camera-payload UART) |

The over-the-air `RAW_SENSOR` layout is packed in `tbeam/apps/satellite/main/frame.c`.
Field order, scale factors and sentinels are documented in `frame.h`.

## Build

You need ESP-IDF **v5.4.1**.

```bash
git clone --recurse-submodules https://github.com/alexandergmzx/swarm-node-firmware.git
cd swarm-node-firmware/tbeam/apps/satellite
. ~/esp/v5.4.1/esp-idf/export.sh

# one build directory per unit; the node id (1-9, the 500 ms TDMA slot) comes from an overlay
idf.py -B build-node1 -DSDKCONFIG=$PWD/build-node1/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;nodes/node1.defaults" -DIDF_TARGET=esp32 build
idf.py -B build-node1 -p /dev/ttyACM0 flash monitor
```

The receiver and the bring-up probe have no per-unit overlays: build them from their own app directory with the same `-B`/`-DIDF_TARGET` arguments and no `nodes/` file.
Per-unit provisioning, the task and core map, the bus maps and the radio profile are described in
[`tbeam/README.md`](tbeam/README.md).

## Radio profiles

All nodes of one mesh must share a profile. The band is a build choice, because the RF front end
cannot be read by firmware:

| Profile | Radio | Boards | TX power range |
|---|---|---|---|
| `433-LAB` | 433.175 MHz / 125 kHz / SF7 | classic T-Beam; 433 MHz S3 Supreme | 2–10 dBm (EU SRD: 10 mW e.r.p.) |
| `915-MX` | 916.700 MHz / 500 kHz / SF9, one fixed channel | 915 MHz S3 Supreme | −9 to +22 dBm |

Both use CR 4:5, explicit header, CRC, an 8-symbol preamble and sync word `0x12`. Every build
defaults to +2 dBm. A classic defaults to `433-LAB`. An S3 build names its band with
`../../profiles/band-433.defaults` or `band-915.defaults` and does not compile without one.
`915-MX` uses a 500 kHz channel because Mexico's IFT-008-2015 (§4.3) requires a 6 dB bandwidth of
at least 500 kHz for a non-hopping system in 902–928 MHz. Its centre frequency is provisional
until a noise scan confirms it. Check the rules of your country before you transmit.

## Cross-references

Comments and READMEs cite the project's design record:

- "ADR NNNN" means an architecture decision record.
- Paths under `docs/`, `protocols/`, `hardware/`, `python-scripts/` and `tools/` refer to files in
  the parent repository.

Neither is needed to build or run the firmware.

## Licence

MIT — see [`LICENSE`](LICENSE). The libraries under `tbeam/third_party/` keep their own licences.
