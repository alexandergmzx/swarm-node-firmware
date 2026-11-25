# `radio_lab` — field radio tests on the T-Beam S3 Supreme

A test bench, not a node app. It measures how far two Supremes reach each other (ping-pong, with GPS
distance), listens to other LoRa radios, transmits tagged bursts for them to count, and sweeps the
noise floor across the band. Every transmission stays under the build profile's airtime ceiling
(1 % at `915-MX`), and only settings the band allows are transmitted. Listening works at any setting.

## Build and flash

```sh
cd tbeam/apps/radio_lab
idf.py -B build-915 -DSDKCONFIG=$PWD/build-915/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;../../profiles/band-915.defaults" -DIDF_TARGET=esp32s3 build
idf.py -B build-915 -p /dev/ttyACM<n> flash monitor
```

A board fresh from the factory needs ROM download mode once: hold BOOT, tap RST, release BOOT.

Without ESP-IDF: every CI run uploads a ready image (`radio_lab-esp32s3-915`, file `merged-flash.bin`).
Flash it with esptool alone: `esptool --chip esp32s3 --port /dev/ttyACM<n> write-flash 0x0 merged-flash.bin`.
Back up a factory image first if you want it back: `esptool --port <port> --before no-reset read-flash 0 ALL factory.bin`.

The parent project's ground station has a web page for this app (its `#/lab` route): it shows the results
live and sends these same console commands. It talks to the board through the ground station or straight
from Chrome or Edge over USB.

## Controls

The board boots in **LISTEN**, which never transmits. Fit a 915 MHz antenna before choosing a mode
that sends.

| Input | Does |
|---|---|
| BOOT, short press | next mode: LISTEN → PING → PONG → SWEEP → LISTEN |
| BOOT, long press (≥ 0.8 s) | next preset (below) |
| USB console | `help`, `status`, `mode <m>`, `preset <n>`, `set freq\|bw\|sf\|cr\|sync\|pwr <v>`, `tx <N>` (LISTEN only) |

**Presets.** All use the fleet channel: 916.700 MHz, 500 kHz, CR 4:5, sync `0x12`. **Both boards must show
the same preset**, because a different SF means they cannot hear each other.

| Preset | SF | TX power | 18 B frame on air | Ping period at 1 % |
|---|---|---|---|---|
| P0 | 8 | +2 dBm | 23 ms | 2.4 s |
| P1 | 8 | +14 dBm | 23 ms | 2.4 s |
| P2 | 8 | +20 dBm | 23 ms | 2.4 s |
| P3 | 10 | +20 dBm | ≈ 80 ms | ≈ 8 s |
| P4 | 12 | +20 dBm | ≈ 300 ms | ≈ 30 s |

The P0–P2 airtime and period are measured on the bench (RadioLib `getTimeOnAir`: 23 168 µs, 2 366 ms period). The P3 and P4 figures are
the Semtech formula, and the board prints the exact value in its `CFG` line.

## A range test with two Supremes

1. **Base:** one board stays put, best on a laptop running `idf.py monitor` (or any serial logger), in
   **PING**.
2. **Walker:** the other board, on its battery, in **PONG**. It needs no laptop: the screen shows what it
   hears.
3. Wait for a GPS fix on both boards (near a window or outside). Without both fixes the distance reads
   `no GPS fix`/`peer has no fix`, but loss and RSSI still count.
4. Walk outwards and stop at each point for at least 20 pings. At P0–P2 that is about a minute. Read the
   base screen (`sent`, `back`, `loss`, `fwd`/`bck` RSSI and SNR, `dist`), or let the laptop log it.
5. When the link drops, long-press both boards to the next preset, from P0 up to P4, and repeat. Every
   preset or settings change starts fresh counters. If the two boards are on different presets or powers
   the base shows `PRESET?` and the walker `PEER PRESET DIFFERS`.

Screen in PING (base):

```
PING P2 SF8 +20dBm       mode, preset, SF, power (title bar)
916.700 500k             channel; "RX ONLY" when the settings may not transmit
sent 42 back 40          pings sent, pongs received
loss 4%
fwd -97dBm 6.5dB         walker's RSSI/SNR of our ping
bck -95dBm 7.0dB 1s      our RSSI/SNR of its pong, age
dist 412 m               GPS distance between the boards
every 2.4s air<=1%
```

## With other people's LoRa radios (raw SX127x / SX126x / RadioLib)

Set their radio to the **fleet channel** to hear our boards:

| Parameter | Value |
|---|---|
| Frequency | 916.700 MHz |
| Bandwidth | 500 kHz |
| Spreading factor | 8 (P0–P2), 10 (P3), 12 (P4) |
| Coding rate | 4/5 |
| Sync word | `0x12`: private. RadioLib on SX126x translates it to `0x1424`; on SX127x write `0x12` to RegSyncWord |
| Preamble | 8 symbols |
| Header / CRC | explicit header, CRC on |
| IQ | normal (not inverted) |

- **They listen to us.** Put our board in LISTEN and send `tx 50` (bursts run in LISTEN only, so they never take a ping's or pong's airtime). It sends 50 tagged frames,
  18 bytes starting `53 4c 03` (`"SL"`, type 3) followed by a 16-bit little-endian sequence number, paced to the
  airtime ceiling. They count the frames they receive and note RSSI/SNR.
- **We listen to them.** Put our board in LISTEN with their settings, for example `set freq 915.0`,
  `set bw 125`, `set sf 7`, `set sync 34`. Every packet prints as an `RX` line and shows on the screen.
  Settings that our band rule does not allow for transmitting (in Mexico: anything but a 500 kHz channel
  inside 902–928 MHz) put the board in **RX ONLY**. It then listens but never sends.
- **Noise sweep.** SWEEP measures every 500 kHz channel from 902.25 to 927.75 MHz (peak RSSI per channel,
  bar graph on the screen, `SW` lines on the console). Run it with their radios silent and then
  transmitting, to see which channels they occupy before the fleet channel is confirmed.

## Console lines (CSV)

| Prefix | Fields |
|---|---|
| `CFG` | mode, freq_khz, bw_khz, sf, cr, sync, tx_dbm, `toa_us=`, `period_ms=`, `tx=allowed\|BLOCKED` |
| `PP` (base) | uptime_ms, seq, preset, sf, tx_dbm, `ok`\|`mismatch`, fwd_rssi, fwd_snr, bck_rssi, bck_snr, dist_m, lat, lon, peer_lat, peer_lon, sent, back, peer_preset, peer_dbm — or uptime_ms, seq, preset, sf, tx_dbm, `lost`. `mismatch` means the walker answered from another preset or power |
| `PO` (walker) | uptime_ms, seq, peer_preset, peer_dbm, rssi, snr, `replied\|skipped_ceiling\|tx_error\|heard`, dist_m, lat, lon, peer_lat, peer_lon, `match\|mismatch` |
| `RX` | uptime_ms, rssi, snr, length, hex (first 64 bytes) |
| `SW` | uptime_ms, freq_khz, mean_dbm, peak_dbm |
| `BURST` | uptime_ms, seq |

lat/lon are degrees × 10⁷; `dist_m` is −1 without both fixes.

## Limits

- Distance is great-circle between two GPS fixes, so each fix's error, a few metres, adds to it.
- The PONG is sent about 10 ms after the PING ends. A third board in PONG mode on the same preset would
  answer too and collide, so keep one walker per preset.
- The sweep reads the SX1262's instantaneous RSSI in 500 kHz. It shows relative occupancy, not a
  calibrated spectrum.
