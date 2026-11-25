# `tbeam/` — the prototype fleet's firmware

ESP-IDF v5.4.1 (ADR 0028; confirmed working on
the development machine — a newer v6.0.2 install exists but needs its Python environment repaired
before it is usable, and nothing here requires v6 specifically). Replaces the empty Zephyr
scaffolding that used to live at `firmware/tbeam-s3/`; Zephyr and an all-Supreme fleet return with
the field stage (backlog 0030).

## Two apps, not a role flag

A satellite and the receiver are different programs, not one program with a Kconfig switch. A
satellite is a hard-real-time TDMA transmitter: GPS/NMEA time discipline to find its ~5 ms window
every 5 s, a full Core 1 acquisition pipeline (PMS7003, SCD30, BME688), then a precisely-timed
radio TX on Core 0 — roughly eight tasks across two priority tiers. The receiver (`node_id` 0)
never transmits (ADR 0016's slot 0 stays
silent) and carries no live sensor pipeline this phase
(ADR 0027: its kit is fitted but
idle) — its entire job is continuous-RX radio → one USB tether line per frame, plus one `FIX` line
per superframe saying where it is. No slot to hit, no TDMA state machine; it opens the GNSS UART
for position only (see "The receiver's `FIX` line" below). Forcing that gap into one `main.c` would
optimize for a symmetry the embedded problem does not have, so the two live as separate apps
sharing `components/`:

```
tbeam/
├── components/            # shared: board pins, radio driver, PMU, GPS time/position, protocol wrapper, Bosch driver
│   ├── board/               # per-variant pin/bus headers (board_pins.h)
│   ├── radio/               # RadioLib-behind-a-C-facade (lora_radio.h); the SF7 interop profile lives here
│   ├── pmu/                 # XPowersLib-behind-a-C-facade (pmu.h): AXP2101 rails for the radio and GNSS
│   ├── gps_time/            # NMEA $--RMC time/position estimate + optional PPS (gps_time.h): the satellite's slot clock, the receiver's FIX line
│   ├── swarm_proto/         # INCLUDE_DIRS wrapper over ../../shared/protocol — no header duplication
│   └── bme68x/              # wraps the vendored Bosch driver (no build system of its own)
├── apps/
│   ├── satellite/          # esp32 target (also builds esp32s3, for backlog 0030) — the real-time system
│   ├── receiver/           # esp32s3 target (also builds esp32: a classic as a spare receiver) — the USB-tethered bridge, node_id 0
│   └── board_probe/        # esp32 target only — M1 bring-up diagnostic, not shipped
└── third_party/            # git submodules, pinned (see below)
```

`components/swarm_proto` is a plain `INCLUDE_DIRS` wrapper over `../../shared/protocol` — the
headers are included, never copied, honouring [`shared/README.md`](../shared/README.md)'s rule that directory
stays dependency-free C99. `SWARM_NODE_ID` (per-unit provisioning, default 15 = unprovisioned — the
TDMA task refuses to transmit at 15, mechanically enforcing
`protocols/specs/messages.md`) is a satellite-only Kconfig
knob set per unit through the overlays in `apps/satellite/nodes/` (see "Provisioning a satellite"
below); the receiver's `node_id` is hard-coded 0.

## Build

Each app is its own `idf.py` project root:

```sh
cd tbeam/apps/receiver    # or satellite, or board_probe
# a classic T-Beam (esp32) is always 433 MHz:
idf.py -B build-esp32 -DSDKCONFIG=$PWD/build-esp32/sdkconfig -DIDF_TARGET=esp32 build
# an S3 Supreme (esp32s3) must name its band; a 433 MHz and a 915 MHz Supreme
# are the same chips, so the build refuses to guess:
idf.py -B build-s3-915 -DSDKCONFIG=$PWD/build-s3-915/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;../../profiles/band-915.defaults" -DIDF_TARGET=esp32s3 build
idf.py -B build-s3-915 -p /dev/ttyACM<n> flash monitor
```

### Band profiles

| Profile | Overlay | Radio | Boards | 26 B frame on air |
|---|---|---|---|---|
| `433-LAB` | `profiles/band-433.defaults` (default on `esp32`) | 433.175 MHz / 125 kHz / SF7 | classic T-Beam (SX1278), 433 MHz S3 Supreme | 61.7 ms |
| `915-MX` | `profiles/band-915.defaults` (`esp32s3` only) | 916.700 MHz / 500 kHz / SF9 | 915 MHz S3 Supreme (SX1262) | 51.5 ms |

Both use CR 4:5, an 8-symbol preamble and sync word `0x12`; every node of one mesh must share a
profile. Name build directories after the band (`build-node4-915`): an existing `sdkconfig` wins
over any overlay, so `cmake/band_guard.cmake` stops a build whose overlay disagrees with the band
its `sdkconfig` already records. Before flashing, `grep -E 'SWARM_(NODE_ID|RADIO_PROFILE_NAME)=' <build>/sdkconfig`;
at boot the radio component prints a `PROFILE …` line before it probes the chip.

### Provisioning a satellite (one build directory per unit)

`SWARM_NODE_ID` defaults to 15 = unprovisioned, and a plain `idf.py build` of the satellite
produces a node that never transmits. Each unit gets its own build directory and `sdkconfig`,
seeded from a tracked one-line overlay in [`apps/satellite/nodes/`](apps/satellite/nodes/):

```sh
cd tbeam/apps/satellite
N=1   # node_id 1 → classic-A, 2 → classic-B, 3 → the first S3 satellite …
idf.py -B build-node$N -DSDKCONFIG=$PWD/build-node$N/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;nodes/node$N.defaults" \
       -DIDF_TARGET=esp32 build
idf.py -B build-node$N -p /dev/ttyACM<n> flash monitor

# an S3 Supreme satellite adds its band overlay last:
idf.py -B build-node4-915 -DSDKCONFIG=$PWD/build-node4-915/sdkconfig \
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;nodes/node4.defaults;../../profiles/band-915.defaults" \
       -DIDF_TARGET=esp32s3 build
```

ESP-IDF applies `sdkconfig.defaults.<target>` after each listed file, so the target overlays still
hold. `grep CONFIG_SWARM_NODE_ID build-node$N/sdkconfig` is the check before flashing. Editing
`sdkconfig.defaults` never changes an existing `sdkconfig` — that is why the overlays exist.

`idf.py` acts on the project in its working directory: from anywhere else pass
`-C tbeam/apps/satellite` (a `-B build-node2 flash` issued from another app directory
silently builds and flashes *that* app under a new `build-node2`).

### A classic as a spare receiver

`apps/receiver` also builds for `esp32` (`sdkconfig.defaults.esp32`: console on UART0 through
the CP210x/CH9102 bridge, 4 MB flash). It is the other half of the swap test — the S3 Supreme runs
the satellite build while a classic receives — and a spare receiver for the fleet:

```sh
cd tbeam/apps/receiver
idf.py -B build-esp32 -DSDKCONFIG=$PWD/build-esp32/sdkconfig -DSDKCONFIG_DEFAULTS=sdkconfig.defaults -DIDF_TARGET=esp32 build
```

## Vendored dependencies (`third_party/`, git submodules)

| Library | Pinned at | Why this pin |
|---|---|---|
| [RadioLib](https://github.com/jgromes/RadioLib) | commit `187ef2479` (past the `7.7.1` tag) | The tagged release's bundled ESP-IDF HAL example is ESP32-classic-only (raw-register SPI, `#error`s on other targets). The portable, `driver/spi_master.h`-based HAL — tested in RadioLib's own CI across esp32/esp32s3/esp32c3 — was promoted to a first-party library HAL (`src/hal/ESP-IDF/`, auto-included from `#include <RadioLib.h>` on ESP-IDF builds) on 2026-06-12 and hardened through 2026-07-03, after the last tag. A SHA pin past an unreleased fix is a normal call, not a moving target — "no version moves mid-phase" (ADR 0028) still applies from here. |
| [XPowersLib](https://github.com/lewisxhe/XPowersLib) | `v0.3.3` | One PMU driver for the whole fleet: every board — both classics (bench-identified 2026-09-03) and the S3 Supreme — carries an **AXP2101**; `components/pmu` accepts the AXP2101 only (`begin()` refuses any other chip ID), so a misidentified or replacement unit fails loudly at boot. Wrapped by `components/pmu` (`swarm_pmu_init()`), which enables the ALDO rails the radio and the GNSS hang from — off at power-up on every board — before anything else touches them. |
| [BME68x_SensorAPI](https://github.com/boschsensortec/BME68x_SensorAPI) | `v4.4.8` | Bosch's own reference driver (`components/bme68x` wraps it — plain vendor C, no build system of its own). |

**Not vendored: Sensirion's `embedded-scd`.** It was tried and dropped: its `embedded-common`
dependency is itself an uninitialized nested submodule, and the driver expects a `make release`
step to flatten several repos into one buildable tree — friction that fits `idf_component_register`
poorly for a protocol this small. `apps/satellite/main/sensors/scd30.c` is hand-rolled directly
against the primary source
(`docs/app-notes/sensirion/sensirion_scd30_interface-description_v1.0.pdf`, filed in the parent
repository's reference library since 2026-07-28 — no need to re-acquire it), the same way `pms7003.c`
already was against Plantower's datasheet — five I2C commands and a fifteen-line CRC-8, verified
this session against the datasheet's own worked example (`CRC(0xBEEF) == 0x92`) and the
CRC-8/NRSC-5 catalog check value.

## Bring-up prerequisites (resolved 2026-09-03)

The T-Beam checkup found two things that would have made every M0/M1 probe fail regardless of
wiring; both are closed in the tree now.

1. **PMU rails.** The AXP2101 feeds the radio and the GNSS module from LDOs under firmware
   control — classic: **ALDO2 → LoRa, ALDO3 → GPS** (vendor schematic sheet 5); S3: **ALDO3 →
   radio, ALDO4 → GNSS**, on a *second* I2C bus **SDA IO42 / SCL IO41**. Measured cold on
   classic-A (2026-09-03, `hardware/bringup-log.md`): **ALDO2 is OFF at power-up with a 2.8 V
   setting; ALDO3 is already ON at 3.3 V** — the radio needs the firmware enable, the GNSS does
   not, and the explicit 3.3 V matters. `components/pmu`
   (`pmu.h`: `swarm_pmu_init()`) brings the AXP2101 up over XPowersLib's ESP-IDF path, enables
   this target's two rails at 3.3 V, waits 50 ms, and reports chip ID plus battery/VBUS/system
   voltages. It runs first in `apps/satellite` (on the shared sensor bus), `apps/receiver` (on
   the dedicated PMU bus it opens itself) and `apps/board_probe`. `board_pins.h` carries the PMU
   pins (`SWARM_PIN_PMU_SDA/SCL/IRQ`), `SWARM_PMU_SHARES_SENSOR_BUS`, and `SWARM_I2C_ADDR_OLED`.
   The M0 Arduino sketches `radio_probe` and `gps_probe` (in the parent repository, not included here) do the same two rail writes through XPowersLib's
   Arduino API before touching the radio or the UART; `pmu_probe` stays read-only.
2. **Band.** The interop profile is **433.175 MHz** at **+2 dBm indoors** (ADR 0031):
   `components/radio/lora_radio.cpp` `kFreqMHz`, TX power from `CONFIG_SWARM_TX_POWER_DBM`
   (`components/radio/Kconfig`, range 2–10, default 2; the EU field profile of the ADR 0031
   amendment builds with 8 — `idf.py menuconfig` → "Swarm radio profile", or a line in the app's
   `sdkconfig.defaults`), the `SX1278` RadioLib class on
   the esp32 target (probed first; SX1262 stays as a guard, and the order flips on esp32s3), and
   `radio_probe`'s `PROFILE_FREQUENCY_MHZ`. RadioLib's `SX1278` class rejects anything above
   525 MHz, which is what made the old 915 MHz sketch a guaranteed false negative.

The electrical confirmation per unit is done (2026-09-03, all three boards, plus 10/10 link tests
from each classic to the receiver) — `hardware/bringup-log.md`;
the runbook that produced it is in `tools/bringup/README.md`.

## Task / core map (`apps/satellite`)

FreeRTOS SMP replaces Zephyr's AMP image split (ADR 0028
decision 3) — one kernel, real `xTaskCreatePinnedToCore` affinity. Architecture tenets 1 and 2 (hard real-time on Core 0; blocking sensor I/O on Core 1) hold: the
isolation boundary is core affinity plus priority inside one kernel, not separate images bridged by
IPC.

| Task | Core | Priority | Owns |
|---|---|---|---|
| PPS ISR (if routed) | 0 | ISR | Captures `esp_timer_get_time()` on each PPS edge; tightens `gps_time`'s estimate when fresh |
| `tdma` | 0 | 23 | Sole owner of "may TX now"; packs and transmits one `RAW_SENSOR` per superframe |
| `gps_uart` (`components/gps_time`) | 0 | 18 | NMEA `$--RMC` parsing → the epoch/position estimate `tdma` reads every superframe |
| `packer` (folded into `tdma`) | 0 | 23 | Frame packing happens inline in `tdma_task`, immediately before TX — no separate task |
| `pms7003` | 1 | 5 | UART frame parser, ~30 s warm-up gate |
| `scd30` | 1 | 5 | I2C, 2 s measurement cycle (faster than the 5 s superframe, inside the 6 s staleness window), feeds `shared_state` |
| `bme688` | 1 | 5 | I2C forced-mode reads (T/RH/P + gas), overlays `shared_state`, feeds the SCD30's pressure compensation directly (same bus, no `CALIBRATION_EVENT` round trip) |

`shared_state.c` is the Core 1 → Core 0 hand-off `sensor_bundle_t` has been since
ADR 0023 item 4: acquisition tasks write their own
fields under a critical section; `tdma_task` snapshots once per superframe, applying the staleness
rule from `protocols/specs/messages.md` (a sensor whose setter has not fired in > 6 s has its VALID
bit cleared — validity bits are only ever cleared here, never set).

The receiver (`apps/receiver`) runs its RX loop in the main task: continuous-RX, printing
`RX <rssi> <snr> <hex>` per received frame. No core-pinning subtlety applies — nothing on it has a
deadline shorter than "before the next frame arrives."

### The receiver's `FIX` line

Since 2026-09-12 the receiver also starts `components/gps_time`'s `gps_uart` task — the same NMEA
reader the satellites discipline their slot clock from — and uses it for **position only**: the
ground station's bridge ingest needs to know where each receiver is (its `receiver_fixes` table
and registry), and the S3's own L76K is the cheapest source of that. This is why the receiver now
opens the GNSS UART; the rail is already up (`components/pmu` enables ALDO4 on the S3, ALDO3 on a
classic spare, before the radio probe). Once every 5000 ms — one TDMA superframe — the RX loop
snapshots `swarm_gps_time_get_ref()` and prints

```
FIX <lat> <lon> <epoch_s> <fix> <disc> <err_ms>
```

(`FIX 0.0000000 0.0000000 0 0 0 0` until the first fix; grammar and field table in
`protocols/specs/messages.md`). It is printed from the same
task as the `RX` lines, so the two can never interleave, which is also why the RX poll now times
out every 1000 ms instead of blocking forever: the cadence has to hold through radio silence.
Node 0 still never transmits — the estimate's time half is printed as a clock-quality readout,
never used to schedule anything. Parsers that only read `RX ` lines ignore `FIX` under the
tether's existing rule.

## Bus map

Bus addresses are identical on both variants: PMU `0x34`, SCD30 `0x61`, BME688 `0x76` (the S3's
onboard BME280 owns `0x77` on the same class of bus, per
ADR 0014, so the external part is strapped to
`0x76` fleet-wide). Pin numbers are per-variant; the machine-readable source of truth is
[`components/board/include/board_pins.h`](components/board/include/board_pins.h), which this table
mirrors.

### T-Beam S3 Supreme (receiver) — confirmed against the vendor datasheet

| Signal | Pin |
|---|---|
| LoRa SCK / MISO / MOSI / CS / RST | IO12 / IO13 / IO11 / IO10 / IO5 |
| LoRa DIO1 / BUSY (SX1262 — no DIO0 on this chip) | IO1 / IO4 |
| I2C SDA / SCL | IO17 / IO18 |
| GPS UART RX / TX / PPS | IO9 / IO8 / IO6 (do not reuse — ADR 0023) |
| PMS7003 RX / TX / `SET` | IO39 / IO38 / IO2 |

### Classic T-Beam (satellite) — confirmed on the bench (2026-09-03)

Authoritative reference: the vendor V1.2 schematic and its notes file,
`docs/datasheets/lilygo/t-beam-classic/` — both
bench units are that one design (title block "T22_GPS_V1.1 replaced with AXP2101"). Identity:
PMU **AXP2101**, GPS **NEO-6M**, radio **SX1278** (HPDTek HPD14A module, 433 MHz), OLED **SSD1306
`0x3C`** on both. Confirmed electrically on both units on 2026-09-03, including a 10/10 link test to the receiver
(`hardware/bringup-log.md`); `board_pins.h` carries the confirmed values.

| Signal | Pin |
|---|---|
| LoRa SCK / MISO / MOSI / CS | 5 / 19 / 27 / 18 |
| LoRa RST | **23** (schematic sheet 1, `IO23=IRQ/RESET`) |
| LoRa DIO0 / DIO1 / DIO2 | 26 / 33 / 32 (SX1278: DIO0 is the TX/RX-done IRQ; no BUSY line exists on this part) |
| I2C SDA / SCL | 21 / 22 — PMU `0x34`, OLED `0x3C`, SCD30 `0x61`, BME688 `0x76` |
| GPS UART RX / TX / PPS | 34 / 12 / **none** — TIME_PULSE drives only the fix LED (schematic sheet 3); `SWARM_PIN_GPS_PPS` stays −1 |
| PMU rails | ALDO2 → LoRa, ALDO3 → GPS — both enabled at 3.3 V at boot by `components/pmu` |
| PMS7003 RX / TX / `SET` | 36 / 13 / 15 |

**The LoRa RST dispute is closed.** Meshtastic's `variant.h` (GPIO 23) matched the vendor
schematic; RadioLib's upstream ESP-IDF example (GPIO 14) did not. GPIO 14 is therefore free — the
one candidate for a PPS bodge wire from the module side of R34, should PPS discipline ever be
wanted on a classic (ADR 0027 second
bring-up note).

## Interop profile (`components/radio`)

Every node on the mesh must agree on all seven parameters — this is the one place they are named
as code, mirroring ADR 0016 /
ADR 0029:

**433.175 MHz** (ADR 0031 — every board on hand is a
433 MHz variant) · 125 kHz · SF7 · CR 4:5 ·
explicit header · PHY CRC on · 8-symbol preamble · private sync word `0x12` · **TX power from
`CONFIG_SWARM_TX_POWER_DBM`** — 2 dBm indoors (the minimum shared by both parts; 433 MHz is not a
free-use band in Mexico, so lab tests there are indoor only) and 8 dBm for the EU field profile
(≤ 10 mW e.r.p. with the stock whip — ADR 0031 amendment, 2026-09-05); the Kconfig range stops at
10 so no build exceeds the EU cap. The other six are constants: the mesh must agree on them. On
the SX1262 side (the S3), `DIO2`-as-RF-switch and 1.8 V TCXO match the glue LilyGO's SX1262 module
uses (Meshtastic's `variant.h`: `SX126X_DIO2_AS_RF_SWITCH`, `SX126X_DIO3_TCXO_VOLTAGE 1.8`). The
classic satellites' radio is settled — SX1278 on the HPDTek HPD14A module — so the boot-time
SX1278-then-SX1262 probe in `lora_radio_init()` (reversed on esp32s3) is now a guard rather than a discovery: RadioLib's
`begin()` performs the SX127x `RegVersion == 0x12` check itself. The vendor-correct class for this
part is `SX1278` (RadioLib restricts it to 137–175 / 395–525 MHz; `SX1276` also accepts 433 MHz).
