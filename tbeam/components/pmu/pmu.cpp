// SPDX-License-Identifier: MIT
//
// C facade over XPowersLib's AXP2101 driver (vendored, v0.3.3). The ESP-IDF
// >= 5 path (CONFIG_XPOWERS_ESP_IDF_NEW_API, the Kconfig default) takes an
// i2c_master bus handle, so the PMU can share the sensor bus on the classic
// T-Beam and own its dedicated bus on the S3 Supreme.

#include "pmu.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

#include "board_pins.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pmu";

static XPowersPMU s_pmu;
static i2c_master_bus_handle_t s_bus = nullptr;
static bool s_up = false;

static esp_err_t open_pmu_bus() {
  i2c_master_bus_config_t cfg = {};
  cfg.i2c_port = -1;
  cfg.sda_io_num = static_cast<gpio_num_t>(SWARM_PIN_PMU_SDA);
  cfg.scl_io_num = static_cast<gpio_num_t>(SWARM_PIN_PMU_SCL);
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.glitch_ignore_cnt = 7;
  cfg.flags.enable_internal_pullup = true;
  return i2c_new_master_bus(&cfg, &s_bus);
}

static void fill(swarm_pmu_info_t *out) {
  if (out == nullptr) return;
  out->chip_id = s_pmu.getChipID();
  out->battery_present = s_pmu.isBatteryConnect();
  out->vbus_present = s_pmu.isVbusIn();
  out->charging = s_pmu.isCharging();
  out->batt_mv = s_pmu.getBattVoltage();
  out->vbus_mv = s_pmu.getVbusVoltage();
  out->sys_mv = s_pmu.getSystemVoltage();
}

extern "C" esp_err_t swarm_pmu_init(i2c_master_bus_handle_t shared_bus, swarm_pmu_info_t *out_info) {
  if (s_up) {
    fill(out_info);
    return ESP_OK;
  }

  if (SWARM_PMU_SHARES_SENSOR_BUS && shared_bus != nullptr) {
    s_bus = shared_bus;
  } else {
    esp_err_t err = open_pmu_bus();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "PMU bus open failed on SDA=%d SCL=%d: %s", SWARM_PIN_PMU_SDA,
               SWARM_PIN_PMU_SCL, esp_err_to_name(err));
      return err;
    }
  }

  // begin() reads the chip ID and refuses anything but an AXP2101.
  if (!s_pmu.begin(s_bus, AXP2101_SLAVE_ADDRESS)) {
    ESP_LOGE(TAG, "no AXP2101 at 0x%02x on SDA=%d SCL=%d", AXP2101_SLAVE_ADDRESS,
             SWARM_PIN_PMU_SDA, SWARM_PIN_PMU_SCL);
    return ESP_ERR_NOT_FOUND;
  }

  // The rails behind the radio and the GNSS module — vendor schematics
  // (classic sheet 5; S3 hardware page). 3.3 V, the same values LilyGO's own
  // examples and Meshtastic's Power.cpp program. Nothing else is touched:
  // DC1 (the MCU) is on already, and the S3's ALDO1 (onboard sensors, OLED)
  // stays whatever the vendor default is this phase.
  //
  // The state as found is logged first: after a cold start it is the chip's
  // power-on default (classic-A measured 2026-09-03: ALDO2 off at 2.8 V,
  // ALDO3 on at 3.3 V); after a warm reset it is whatever ran before, because
  // the PMU keeps its registers while it has power.
#if CONFIG_IDF_TARGET_ESP32
  ESP_LOGI(TAG, "as found: ALDO2 (LoRa) %s %u mV, ALDO3 (GPS) %s %u mV",
           s_pmu.isEnableALDO2() ? "on" : "OFF", s_pmu.getALDO2Voltage(),
           s_pmu.isEnableALDO3() ? "on" : "OFF", s_pmu.getALDO3Voltage());
  s_pmu.setALDO2Voltage(3300);
  s_pmu.enableALDO2();   // LORA_VCC
  s_pmu.setALDO3Voltage(3300);
  s_pmu.enableALDO3();   // GPS_VDD
  const char *rails = "ALDO2 (LoRa) + ALDO3 (GPS)";
#else
  ESP_LOGI(TAG, "as found: ALDO3 (radio) %s %u mV, ALDO4 (GNSS) %s %u mV",
           s_pmu.isEnableALDO3() ? "on" : "OFF", s_pmu.getALDO3Voltage(),
           s_pmu.isEnableALDO4() ? "on" : "OFF", s_pmu.getALDO4Voltage());
  s_pmu.setALDO3Voltage(3300);
  s_pmu.enableALDO3();   // radio
  s_pmu.setALDO4Voltage(3300);
  s_pmu.enableALDO4();   // GNSS
  const char *rails = "ALDO3 (radio) + ALDO4 (GNSS)";
#endif
  // Let the radio and the GNSS module come out of power-on reset on the
  // fresh rail before anybody talks to them.
  vTaskDelay(pdMS_TO_TICKS(50));

  s_pmu.enableBattDetection();
  s_pmu.enableBattVoltageMeasure();
  s_pmu.enableVbusVoltageMeasure();
  s_pmu.enableSystemVoltageMeasure();

  s_up = true;
  swarm_pmu_info_t info;
  fill(&info);
  if (out_info) *out_info = info;
  ESP_LOGI(TAG, "AXP2101 (id 0x%02x) up, %s at 3.3 V; batt %u mV%s, vbus %u mV%s, sys %u mV",
           info.chip_id, rails, info.batt_mv, info.battery_present ? "" : " (no cell)",
           info.vbus_mv, info.vbus_present ? "" : " (absent)", info.sys_mv);
  return ESP_OK;
}

extern "C" i2c_master_bus_handle_t swarm_pmu_bus(void) { return s_bus; }
