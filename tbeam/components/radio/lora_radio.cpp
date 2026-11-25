// SPDX-License-Identifier: MIT
//
// C facade over RadioLib. See lora_radio.h for the contract; this file owns
// the C++ RadioLib objects (EspHal + SX1278/SX1262) and never leaks a C++
// type across the component boundary.

#include "lora_radio.h"

#include <RadioLib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "board_pins.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "lora_radio";

// Interop profile shared by every node on the mesh (ADR 0016 / ADR 0029 /
// ADR 0031). Both radios must agree on all of these; this is the one place
// they are named. Frequency, bandwidth and SF come from the profile choice in
// Kconfig (433-LAB or 915-MX); an esp32s3 build without a band overlay stops
// here, because firmware cannot tell a 433 MHz Supreme from a 915 MHz one.
#if CONFIG_SWARM_RADIO_PROFILE_UNSET
#error "No radio profile: delete this build directory's sdkconfig, then add ../../profiles/band-433.defaults or band-915.defaults to SDKCONFIG_DEFAULTS"
#elif CONFIG_SWARM_RADIO_PROFILE_915 && CONFIG_IDF_TARGET_ESP32
#error "915-MX profile on esp32: the classic T-Beam's SX1278 stops at 525 MHz"
#endif
namespace {
constexpr float kFreqMHz = CONFIG_SWARM_RADIO_FREQ_KHZ / 1000.0f;
constexpr float kBandwidthKHz = CONFIG_SWARM_RADIO_BW_KHZ;
constexpr uint8_t kSpreadingFactor = CONFIG_SWARM_RADIO_SF;
constexpr uint8_t kCodingRateDenom = 5;   // CR 4:5, RadioLib's "cr" is the denominator
constexpr uint8_t kSyncWord = 0x12;       // private — RadioLib translates this for SX126x internally
constexpr int8_t kTxPowerDbm = CONFIG_SWARM_TX_POWER_DBM;  // 2 = bench default; range per profile in Kconfig
constexpr uint16_t kPreambleSymbols = 8;
constexpr float kSx1262TcxoVoltage = 1.8f;
// SX1262 over-current protection. RadioLib's begin() leaves 60 mA, which
// clips the PA well below +22 dBm; LilyGO's factory sketch and Meshtastic
// both set 140 mA, the datasheet value for the SX1262's high-power PA.
constexpr float kSx1262CurrentLimitMa = 140.0f;
}  // namespace

static EspHal *s_hal = nullptr;
static PhysicalLayer *s_radio = nullptr;  // polymorphic: either an SX1278 or an SX1262
static Module *s_module = nullptr;
static lora_chip_t s_chip = LORA_CHIP_UNKNOWN;
static SemaphoreHandle_t s_rx_sem = nullptr;

// Fires from RadioLib's ISR context (attached to the chip's IRQ pin by
// startReceive()) — must stay ISR-safe: no logging, no blocking calls.
static void IRAM_ATTR rx_isr(void) {
  BaseType_t woken = pdFALSE;
  if (s_rx_sem != nullptr) {
    xSemaphoreGiveFromISR(s_rx_sem, &woken);
  }
  if (woken) {
    portYIELD_FROM_ISR();
  }
}

// SX1278: the 137–525 MHz member of the SX127x family, which is what the
// classic T-Beam's HPD14A module carries. RadioLib's SX1278 class rejects
// anything above 525 MHz, so it also enforces that the profile stays inside
// the band the module is matched for.
[[maybe_unused]] static esp_err_t try_sx1278() {
  s_module = new Module(s_hal, SWARM_PIN_LORA_CS, SWARM_PIN_LORA_DIO0,
                         SWARM_PIN_LORA_RST, SWARM_PIN_LORA_DIO1);
  auto *sx1278 = new SX1278(s_module);
  int16_t state = sx1278->begin(kFreqMHz, kBandwidthKHz, kSpreadingFactor,
                                 kCodingRateDenom, kSyncWord, kTxPowerDbm,
                                 kPreambleSymbols);
  if (state == RADIOLIB_ERR_NONE) {
    s_radio = sx1278;
    s_chip = LORA_CHIP_SX1278;
    return ESP_OK;
  }
  delete sx1278;
  delete s_module;
  s_module = nullptr;
  if (state != RADIOLIB_ERR_CHIP_NOT_FOUND) {
    ESP_LOGW(TAG, "SX1278 probe: begin() failed with code %d (not a not-found result)", state);
  }
  return ESP_ERR_NOT_FOUND;
}

static esp_err_t try_sx1262() {
  // SX126x Module pin order is (cs, dio1, nrst, busy) — DIO1 is the primary
  // IRQ line on this family, unlike SX127x's DIO0.
  s_module = new Module(s_hal, SWARM_PIN_LORA_CS, SWARM_PIN_LORA_DIO1,
                         SWARM_PIN_LORA_RST, SWARM_PIN_LORA_BUSY);
  auto *sx1262 = new SX1262(s_module);
  int16_t state = sx1262->begin(kFreqMHz, kBandwidthKHz, kSpreadingFactor,
                                 kCodingRateDenom, kSyncWord, kTxPowerDbm,
                                 kPreambleSymbols, kSx1262TcxoVoltage);
  if (state != RADIOLIB_ERR_NONE) {
    if (state != RADIOLIB_ERR_CHIP_NOT_FOUND) {
      ESP_LOGW(TAG, "SX1262 probe: begin() failed with code %d (not a not-found result)", state);
    }
    delete sx1262;
    delete s_module;
    s_module = nullptr;
    return ESP_ERR_NOT_FOUND;
  }
  // DIO2-as-RF-switch + the TCXO voltage above match LilyGO's SX1262 module
  // glue (Meshtastic variant.h; ADR 0028).
  int16_t ocp = sx1262->setCurrentLimit(kSx1262CurrentLimitMa);
  if (ocp != RADIOLIB_ERR_NONE) {
    ESP_LOGW(TAG, "SX1262: setCurrentLimit(%.0f mA) failed with code %d", kSx1262CurrentLimitMa, ocp);
  }
  int16_t rf_switch = sx1262->setDio2AsRfSwitch(true);
  if (rf_switch != RADIOLIB_ERR_NONE) {
    ESP_LOGW(TAG, "SX1262: setDio2AsRfSwitch failed with code %d — TX may fail into an open switch",
             rf_switch);
  }
  s_radio = sx1262;
  s_chip = LORA_CHIP_SX1262;
  return ESP_OK;
}

extern "C" esp_err_t lora_radio_init(lora_chip_t *out_chip) {
  if (s_radio != nullptr) {
    if (out_chip) *out_chip = s_chip;
    return ESP_OK;  // already initialized
  }

  s_rx_sem = xSemaphoreCreateBinary();
  if (s_rx_sem == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  // Printed before the probe so it shows even when no radio answers: the
  // band is a build choice the board cannot confirm, and a wrong-band build
  // on a node looks like silence, not like an error.
  ESP_LOGW(TAG, "PROFILE %s: %.3f MHz / %d kHz / SF%u / CR4:%u / sync 0x%02x / %+d dBm / 26 B = %.1f ms on air, "
           "%.2f %% of a 5 s superframe (ceiling %.1f %%)",
           CONFIG_SWARM_RADIO_PROFILE_NAME, kFreqMHz, CONFIG_SWARM_RADIO_BW_KHZ, kSpreadingFactor,
           kCodingRateDenom, kSyncWord, kTxPowerDbm, LORA_RADIO_AIRTIME_US(26) / 1000.0,
           LORA_RADIO_AIRTIME_US(26) / 50000.0, CONFIG_SWARM_RADIO_MAX_DUTY_PERMILLE / 10.0);

  s_hal = new EspHal(SWARM_PIN_LORA_SCK, SWARM_PIN_LORA_MISO, SWARM_PIN_LORA_MOSI);

  // The expected part goes first (RadioLib's begin() does the RegVersion /
  // GetStatus identity check itself); the other family stays as a guard so a
  // replacement board of the other variant is still recognised, not silent.
#if CONFIG_IDF_TARGET_ESP32
  esp_err_t err = try_sx1278();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "SX1278 not found (rail on? RST=%d?), probing SX1262 as a guard", SWARM_PIN_LORA_RST);
    err = try_sx1262();
  }
#elif CONFIG_SWARM_RADIO_PROFILE_915
  // No SX1278 guard: the SX1278 class rejects 916.7 MHz, so the guard could
  // only ever report a misleading "begin() failed".
  esp_err_t err = try_sx1262();
#else
  esp_err_t err = try_sx1262();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "SX1262 not found (ALDO3 on?), probing SX1278 as a guard");
    err = try_sx1278();
  }
#endif
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "no supported radio responded on the LoRa SPI bus");
    return ESP_ERR_NOT_FOUND;
  }

  s_radio->setPacketReceivedAction(rx_isr);
  ESP_LOGI(TAG, "radio up: %s at %.3f MHz / %d kHz / SF%u / CR4:%u / sync 0x%02x / %d dBm (%s)",
           s_chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262",
           kFreqMHz, CONFIG_SWARM_RADIO_BW_KHZ, kSpreadingFactor, kCodingRateDenom, kSyncWord,
           kTxPowerDbm, CONFIG_SWARM_RADIO_PROFILE_NAME);

  if (out_chip) *out_chip = s_chip;
  return ESP_OK;
}

lora_chip_t lora_radio_chip(void) { return s_chip; }

extern "C" esp_err_t lora_radio_set_params(const lora_params_t *p) {
  if (s_radio == nullptr || p == nullptr) return ESP_ERR_INVALID_STATE;
  if (s_chip != LORA_CHIP_SX1262) return ESP_ERR_NOT_SUPPORTED;
  auto *r = static_cast<SX1262 *>(s_radio);
  r->standby();
  int16_t st = r->setFrequency(p->freq_khz / 1000.0f);
  if (st == RADIOLIB_ERR_NONE) st = r->setBandwidth((float)p->bw_khz);
  if (st == RADIOLIB_ERR_NONE) st = r->setSpreadingFactor(p->sf);
  if (st == RADIOLIB_ERR_NONE) st = r->setCodingRate(p->cr_denom);
  if (st == RADIOLIB_ERR_NONE) st = r->setSyncWord(p->sync_word);
  if (st == RADIOLIB_ERR_NONE) st = r->setOutputPower(p->tx_dbm);
  if (st == RADIOLIB_ERR_NONE) st = r->setCurrentLimit(kSx1262CurrentLimitMa);  // setOutputPower keeps OCP, but be explicit
  if (st != RADIOLIB_ERR_NONE) {
    ESP_LOGW(TAG, "set_params %lu kHz / %u kHz / SF%u / CR4:%u / 0x%02x / %d dBm rejected, code %d",
             (unsigned long)p->freq_khz, p->bw_khz, p->sf, p->cr_denom, p->sync_word, p->tx_dbm, st);
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

extern "C" uint32_t lora_radio_time_on_air_us(size_t len) {
  if (s_radio == nullptr) return 0;
  return (uint32_t)s_radio->getTimeOnAir(len);
}

extern "C" esp_err_t lora_radio_rssi_now(float *out_dbm) {
  if (s_radio == nullptr || out_dbm == nullptr) return ESP_ERR_INVALID_STATE;
  if (s_chip != LORA_CHIP_SX1262) return ESP_ERR_NOT_SUPPORTED;
  *out_dbm = static_cast<SX1262 *>(s_radio)->getRSSI(false);
  return ESP_OK;
}

const lora_profile_t *lora_radio_profile(void) {
  static const lora_profile_t profile = {
      CONFIG_SWARM_RADIO_PROFILE_NAME, CONFIG_SWARM_RADIO_FREQ_KHZ, CONFIG_SWARM_RADIO_BW_KHZ,
      kSpreadingFactor, kCodingRateDenom, kSyncWord, kPreambleSymbols, kTxPowerDbm,
  };
  return &profile;
}

esp_err_t lora_radio_tx(const uint8_t *data, size_t len) {
  if (s_radio == nullptr) return ESP_ERR_INVALID_STATE;
  int16_t state = s_radio->transmit(const_cast<uint8_t *>(data), len);
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGW(TAG, "transmit failed with code %d", state);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t lora_radio_rx_start(void) {
  if (s_radio == nullptr) return ESP_ERR_INVALID_STATE;
  xSemaphoreTake(s_rx_sem, 0);  // drain any stale signal before arming
  int16_t state = s_radio->startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGE(TAG, "startReceive failed with code %d", state);
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t lora_radio_rx_poll(uint8_t *buf, size_t buf_len, size_t *out_len,
                              float *out_rssi, float *out_snr,
                              uint32_t timeout_ms) {
  if (s_radio == nullptr) return ESP_ERR_INVALID_STATE;
  TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  if (xSemaphoreTake(s_rx_sem, ticks) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  size_t pkt_len = s_radio->getPacketLength();
  if (pkt_len > buf_len) {
    ESP_LOGW(TAG, "dropped %u B packet, buffer only holds %u B", (unsigned)pkt_len, (unsigned)buf_len);
    lora_radio_rx_start();  // re-arm before returning so RX-continuous never stalls
    return ESP_ERR_INVALID_SIZE;
  }

  int16_t state = s_radio->readData(buf, pkt_len);
  float rssi = s_radio->getRSSI();
  float snr = s_radio->getSNR();
  lora_radio_rx_start();  // re-arm for continuous RX regardless of outcome

  if (state != RADIOLIB_ERR_NONE) {
    ESP_LOGW(TAG, "readData failed with code %d", state);
    return ESP_FAIL;
  }
  if (out_len) *out_len = pkt_len;
  if (out_rssi) *out_rssi = rssi;
  if (out_snr) *out_snr = snr;
  return ESP_OK;
}
