// adsb-monitor — local flight radar for the Freenove FNK0104S.
//
// Wi-Fi: ESP-Touch v2 provisioning on first boot (mianesp WiFiManager), then SNTP.
// Data: polls the selected provider (adsb.fi by default, or a local readsb) for aircraft
//       within the configured radius of home (Feed.cpp).
// Screen: radar scope plus nearest-aircraft list (Ui.cpp), LVGL on the ST7796.
// BOOT key: hold 3 s within a minute of power-on to clear Wi-Fi and reprovision.
// Settings: web page at /, or GET/POST /config. OTA: POST the .bin to /firmware.

#include <atomic>
#include <cstdlib>
#include <string>

#include "Board.h"
#include "Display.h"
#include "Feed.h"
#include "LogBuffer.h"
#include "NvsStorageManager.h"
#include "Settings.h"
#include "Ui.h"
#include "WebApi.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "wifimanager.h"

namespace {

const char* TAG = "adsb";

constexpr TickType_t kResetHold = pdMS_TO_TICKS(3000);
// Wi-Fi reset by the BOOT key is only armed just after power-on, so a stuck key
// while running can't wipe the credentials.
constexpr TickType_t kResetWindow = pdMS_TO_TICKS(60000);
constexpr TickType_t kOtaVerifyTimeout = pdMS_TO_TICKS(120000);
// WiFiManager retries on every disconnect, but has been seen to stay off the
// network; a software restart recovers it and keeps the RTC log for /log?prev.
constexpr TickType_t kOfflineRestart = pdMS_TO_TICKS(3 * 60 * 1000);

std::atomic<ui::Net> s_net{ui::Net::Connecting};

void applyTz(const std::string& tz) {
  setenv("TZ", tz.c_str(), 1);
  tzset();
}

// NvsStorageManager treats these as success without erasing, leaving NVS
// uninitialised (e.g. a partition left over from previous firmware).
void initNvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS partition unusable (%s), erasing", esp_err_to_name(err));
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

void onIpEvent(void*, esp_event_base_t base, int32_t id, void*) {
  if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) s_net = ui::Net::Online;
}

void onWifiEvent(void*, esp_event_base_t, int32_t id, void* data) {
  if (id != WIFI_EVENT_STA_DISCONNECTED) return;
  const auto* ev = static_cast<const wifi_event_sta_disconnected_t*>(data);
  ESP_LOGW(TAG, "Wi-Fi disconnected, reason %d, rssi %d", ev->reason, ev->rssi);
  ui::Net online = ui::Net::Online;
  s_net.compare_exchange_strong(online, ui::Net::Connecting);
}

bool otaPendingVerify() {
  esp_ota_img_states_t state;
  return esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
         state == ESP_OTA_IMG_PENDING_VERIFY;
}

void initBootKey() {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << board::kBootKey;
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&io));
}

}  // namespace

extern "C" void app_main(void) {
  LogBuffer::install();
  initNvs();
  static NvsStorageManager nvs;
  static Settings settings(nvs);
  settings.log();
  applyTz(settings.tz);

  display::init();
  display::setBrightness(settings.brightness);
  const feed::Config feedCfg = settings.feedConfig();
  ui::init(feedCfg.lat, feedCfg.lon, feedCfg.radiusNm);
  ui::setNet(ui::Net::Connecting, "Starting...");
  ui::setGround(ui::Ground(settings.ground));

  settings.onChange("tz", [] { applyTz(settings.tz); });
  settings.onChange("ground", [] { ui::setGround(ui::Ground(settings.ground)); });
  // Same path as a POST to /config, so the page and the stored value follow the button.
  ui::onGroundChosen([](ui::Ground mode) {
    JsonWrapper change;
    change.AddItem("ground", static_cast<int>(mode));
    settings.loadFromJson(change);
    settings.save();
  });
  settings.onChange("brightness", [] { display::setBrightness(settings.brightness); });
  auto refeed = [] {
    const feed::Config c = settings.feedConfig();
    ui::setHome(c.lat, c.lon, c.radiusNm);
    feed::configure(c);
  };
  for (const char* key : {"latitude", "longitude", "radius_nm", "provider", "providers"}) {
    settings.onChange(key, refeed);
  }

  static WiFiManager wifi(nvs, onIpEvent, nullptr);
  // Build-time hostname only seeds a fresh device; after that the page owns it.
  std::string host;
  if (!nvs.retrieve("hostname", host)) {
    host = CONFIG_ADSB_HOSTNAME;
    wifi.configSetHostName(host);
  }
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, onWifiEvent, nullptr));

  static WebContext webCtx(&wifi);
  static WebApi web(&webCtx, settings);
  if (web.start() != ESP_OK) ESP_LOGE(TAG, "web server failed to start, OTA unavailable");

  wifi_config_t sta = {};
  if (esp_wifi_get_config(WIFI_IF_STA, &sta) == ESP_OK && sta.sta.ssid[0] == '\0') {
    s_net = ui::Net::Unprovisioned;
  }

  esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_cfg));

  feed::start(feedCfg);
  initBootKey();

  bool otaPending = otaPendingVerify();
  TickType_t lastOnline = xTaskGetTickCount();
  TickType_t pressStart = 0;
  bool wasPressed = false;

  while (true) {
    const TickType_t tick = xTaskGetTickCount();
    const ui::Net net = s_net;
    feed::pause(net != ui::Net::Online);

    if (net == ui::Net::Online) {
      lastOnline = tick;
    } else if (net == ui::Net::Connecting && tick - lastOnline > kOfflineRestart) {
      ESP_LOGE(TAG, "offline for 3 min, restarting");
      esp_restart();
    }

    if (otaPending && net == ui::Net::Online) {
      ESP_LOGI(TAG, "OTA image verified (got IP), cancelling rollback");
      esp_ota_mark_app_valid_cancel_rollback();
      otaPending = false;
    } else if (otaPending && tick > kOtaVerifyTimeout) {
      ESP_LOGE(TAG, "no IP within 120 s of OTA boot, rolling back");
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }

    const bool pressed = gpio_get_level(board::kBootKey) == 0;
    if (pressed && !wasPressed) pressStart = tick;
    wasPressed = pressed;
    if (pressed && pressStart < kResetWindow && tick - pressStart >= kResetHold) {
      ui::setNet(net, "Wi-Fi reset");
      vTaskDelay(pdMS_TO_TICKS(500));  // let the message draw
      ESP_LOGW(TAG, "BOOT held 3 s: clearing Wi-Fi credentials");
      wifi.clear();  // reboots
    }

    if (net == ui::Net::Unprovisioned) {
      ui::setNet(net, "Set up Wi-Fi with the ESP-Touch app (v2)");
    } else if (net == ui::Net::Connecting && tick < pdMS_TO_TICKS(20000)) {
      ui::setNet(net, "Connecting to Wi-Fi...");
    } else if (net == ui::Net::Connecting && tick < kResetWindow) {
      // Credentials can be stale, e.g. left in NVS by the board's factory firmware.
      ui::setNet(net, "Can't join Wi-Fi.\nTo set it up again, hold BOOT for 3 s.");
    } else if (net == ui::Net::Connecting) {
      ui::setNet(net, "Can't join Wi-Fi.\nTo set it up again, replug, then hold BOOT for 3 s.");
    } else {
      ui::setNet(net, "");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
