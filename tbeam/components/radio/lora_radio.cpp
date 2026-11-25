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
// they are named.
namespace {
constexpr float kFreqMHz = 433.175f;      // lab phase: every board on hand is a 433 MHz variant (ADR 0031)
constexpr float kBandwidthKHz = 125.0f;
constexpr uint8_t kSpreadingFactor = 7;
constexpr uint8_t kCodingRateDenom = 5;   // CR 4:5, RadioLib's "cr" is the denominator
constexpr uint8_t kSyncWord = 0x12;       // private — RadioLib translates this for SX126x internally
constexpr int8_t kTxPowerDbm = CONFIG_SWARM_TX_POWER_DBM;  // 2 = lab floor, indoors (ADR 0031 decision 2);
                                                          // 8 = EU field profile (ADR 0031 amendment) — Kconfig
constexpr uint16_t kPreambleSymbols = 8;
constexpr float kSx1262TcxoVoltage = 1.8f;
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
static esp_err_t try_sx1278() {
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
  ESP_LOGI(TAG, "radio up: %s at %.3f MHz / SF%u / CR4:%u / sync 0x%02x / %d dBm",
           s_chip == LORA_CHIP_SX1278 ? "SX1278" : "SX1262",
           kFreqMHz, kSpreadingFactor, kCodingRateDenom, kSyncWord, kTxPowerDbm);

  if (out_chip) *out_chip = s_chip;
  return ESP_OK;
}

lora_chip_t lora_radio_chip(void) { return s_chip; }

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
