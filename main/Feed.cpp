#include "Feed.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <strings.h>
#include <mutex>

#include "Geo.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace feed {
namespace {

constexpr char TAG[] = "feed";
// About 470 bytes per aircraft; 1MB is well past the busiest 250 nm circle.
constexpr size_t kMaxBody = 1024 * 1024;
constexpr int kMinPollS = 1;  // a local receiver updates every second

std::mutex s_mutex;
Config s_cfg;
std::shared_ptr<const Snapshot> s_latest = std::make_shared<Snapshot>();
uint32_t s_generation = 0;
std::string s_error = "waiting";
bool s_paused = true;
TaskHandle_t s_task = nullptr;
Status s_status;

struct Body {
  std::string data;
  bool overflow = false;
  std::string rateLimit;
  int rateRemaining = -1;
  int retryAfterS = -1;
};

// Only the delta-seconds form of Retry-After; an HTTP date is ignored.
int headerInt(const char* value) {
  char* end = nullptr;
  const long v = std::strtol(value, &end, 10);
  return end != value && v >= 0 ? static_cast<int>(std::min(v, 86400L)) : -1;
}

esp_err_t onHttpEvent(esp_http_client_event_t* evt) {
  auto* body = static_cast<Body*>(evt->user_data);
  if (evt->event_id == HTTP_EVENT_ON_HEADER) {
    if (strcasecmp(evt->header_key, "x-ratelimit-limit") == 0) body->rateLimit = evt->header_value;
    if (strcasecmp(evt->header_key, "x-ratelimit-remaining") == 0) body->rateRemaining = headerInt(evt->header_value);
    if (strcasecmp(evt->header_key, "retry-after") == 0) body->retryAfterS = headerInt(evt->header_value);
    return ESP_OK;
  }
  if (evt->event_id != HTTP_EVENT_ON_DATA) return ESP_OK;
  if (body->data.size() + evt->data_len > kMaxBody) {
    body->overflow = true;
    return ESP_FAIL;
  }
  body->data.append(static_cast<const char*>(evt->data), evt->data_len);
  return ESP_OK;
}

// mianesp's HttpClient doesn't attach a CA bundle, and the public APIs are
// HTTPS-only. The bundle is unused for a plain-HTTP local receiver.
bool httpGet(const std::string& url, const Config& feedCfg, Body& body, int& status) {
  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.timeout_ms = 10000;
  cfg.event_handler = onHttpEvent;
  cfg.user_data = &body;
  cfg.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.buffer_size = 2048;
  esp_http_client_handle_t client = esp_http_client_init(&cfg);
  if (!client) return false;
  esp_http_client_set_header(client, "User-Agent", "adsb-monitor/1 (esp32)");
  for (const auto& [name, value] : feedCfg.headers) {
    esp_http_client_set_header(client, name.c_str(), value.c_str());
  }
  esp_err_t err = esp_http_client_perform(client);
  status = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);
  if (err != ESP_OK) ESP_LOGW(TAG, "GET failed: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

// '@' is ADS-B's "no character", so an unset callsign arrives as "@@@@@@@@".
std::string trimmed(const char* s) {
  std::string out(s ? s : "");
  while (!out.empty() && (out.back() == ' ' || out.back() == '@')) out.pop_back();
  return out;
}

std::string str(const cJSON* obj, const char* key) {
  const cJSON* item = cJSON_GetObjectItemCaseSensitive(obj, key);
  return cJSON_IsString(item) ? trimmed(item->valuestring) : std::string();
}

bool num(const cJSON* obj, const char* key, double& out) {
  const cJSON* item = cJSON_GetObjectItemCaseSensitive(obj, key);
  if (!cJSON_IsNumber(item)) return false;
  out = item->valuedouble;
  return true;
}

bool parseAircraft(const cJSON* ac, const Config& cfg, Aircraft& a) {
  double v;
  if (!num(ac, "lat", a.lat) || !num(ac, "lon", a.lon)) return false;
  a.hex = str(ac, "hex");
  a.flight = str(ac, "flight");
  a.reg = str(ac, "r");
  a.type = str(ac, "t");
  a.squawk = str(ac, "squawk");
  const std::string emergency = str(ac, "emergency");
  a.emergency = !emergency.empty() && emergency != "none";
  a.rotorcraft = str(ac, "category") == "A7";

  // alt_baro is a number, or the string "ground".
  const cJSON* alt = cJSON_GetObjectItemCaseSensitive(ac, "alt_baro");
  if (cJSON_IsString(alt) && std::string(alt->valuestring) == "ground") {
    a.onGround = true;
    a.hasAlt = true;
    a.altFt = 0;
  } else if (cJSON_IsNumber(alt)) {
    a.hasAlt = true;
    a.altFt = alt->valueint;
  } else if (num(ac, "alt_geom", v)) {
    a.hasAlt = true;
    a.altFt = static_cast<int>(v);
  }
  if (num(ac, "track", v) || num(ac, "true_heading", v)) {
    a.hasTrack = true;
    a.track = v;
  }
  if (num(ac, "gs", v)) a.gsKt = v;
  if (num(ac, "seen_pos", v) && v > 0) a.posAgeS = v;
  if (num(ac, "baro_rate", v) || num(ac, "geom_rate", v)) a.vrateFpm = static_cast<int>(v);
  geo::rangeBearing(cfg.lat, cfg.lon, a.lat, a.lon, a.distNm, a.bearing);
  return true;
}

void replaceAll(std::string& s, const std::string& from, const std::string& to) {
  for (size_t at = s.find(from); at != std::string::npos; at = s.find(from, at + to.size())) {
    s.replace(at, from.size(), to);
  }
}

std::string buildUrl(const Config& cfg) {
  char lat[16], lon[16];
  snprintf(lat, sizeof lat, "%.4f", cfg.lat);
  snprintf(lon, sizeof lon, "%.4f", cfg.lon);
  std::string url = cfg.url;
  replaceAll(url, "{lat}", lat);
  replaceAll(url, "{lon}", lon);
  replaceAll(url, "{nm}", std::to_string(cfg.radiusNm));
  return url;
}

void recordFetch(const Body& body, int httpStatus, int64_t startUs) {
  std::lock_guard<std::mutex> g(s_mutex);
  s_status.httpStatus = httpStatus;
  s_status.fetchMs = static_cast<int>((esp_timer_get_time() - startUs) / 1000);
  s_status.rateLimit = body.rateLimit;
  s_status.rateRemaining = body.rateRemaining;
  s_status.retryAfterS = body.retryAfterS;
}

bool poll(const Config& cfg, std::string& error, int& retryAfterS) {
  const std::string url = buildUrl(cfg);
  Body body;
  int status = 0;
  const int64_t start = esp_timer_get_time();
  const bool got = httpGet(url, cfg, body, status);
  recordFetch(body, got ? status : 0, start);
  retryAfterS = body.retryAfterS;
  if (!got) {
    error = body.overflow ? "response too large" : "no connection to " + cfg.provider;
    return false;
  }
  if (status != 200) {
    error = cfg.provider + " HTTP " + std::to_string(status);
    return false;
  }

  cJSON* root = cJSON_ParseWithLength(body.data.data(), body.data.size());
  if (!root) {
    error = "bad JSON from " + cfg.provider;
    return false;
  }
  auto snap = std::make_shared<Snapshot>();
  snap->fetchedUs = esp_timer_get_time();
  // readsb's aircraft.json and adsb.fi say "aircraft"; adsb.lol says "ac".
  const cJSON* list = cJSON_GetObjectItemCaseSensitive(root, "ac");
  if (!list) list = cJSON_GetObjectItemCaseSensitive(root, "aircraft");
  const cJSON* ac = nullptr;
  cJSON_ArrayForEach(ac, list) {
    Aircraft a;
    // A local receiver returns everything it hears, not just the radius.
    if (parseAircraft(ac, cfg, a) && a.distNm <= cfg.radiusNm) snap->aircraft.push_back(std::move(a));
  }
  cJSON_Delete(root);
  std::sort(snap->aircraft.begin(), snap->aircraft.end(),
            [](const Aircraft& x, const Aircraft& y) { return x.distNm < y.distNm; });

  ESP_LOGI(TAG, "%u aircraft, %u bytes in %lld ms", static_cast<unsigned>(snap->aircraft.size()),
           static_cast<unsigned>(body.data.size()), (esp_timer_get_time() - start) / 1000);
  std::lock_guard<std::mutex> g(s_mutex);
  s_latest = std::move(snap);
  ++s_generation;
  s_status.lastOkUs = esp_timer_get_time();
  return true;
}

void task(void*) {
  int failures = 0;
  while (true) {
    Config cfg;
    bool paused;
    {
      std::lock_guard<std::mutex> g(s_mutex);
      cfg = s_cfg;
      paused = s_paused;
    }
    int waitS = std::max(cfg.pollS, kMinPollS);
    if (!paused) {
      std::string error;
      int retryAfterS = -1;
      if (poll(cfg, error, retryAfterS)) {
        failures = 0;
        error.clear();
      } else {
        ESP_LOGW(TAG, "%s", error.c_str());
        // Back off on repeated failures (rate limits, outages) to a minute, never
        // polling faster than configured, and wait out any Retry-After.
        waitS = std::max(waitS, std::min(60, waitS << std::min(++failures, 4)));
        waitS = std::max(waitS, std::min(retryAfterS, 3600));
      }
      std::lock_guard<std::mutex> g(s_mutex);
      s_error = error;
    }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(waitS * 1000));
  }
}

}  // namespace

bool parseProvider(const std::string& table, const std::string& name, Config& cfg, std::string& error) {
  cJSON* root = cJSON_ParseWithLength(table.data(), table.size());
  const cJSON* entry = cJSON_GetObjectItemCaseSensitive(root, name.c_str());
  const cJSON* url = cJSON_GetObjectItemCaseSensitive(entry, "url");
  const cJSON* headers = cJSON_GetObjectItemCaseSensitive(entry, "headers");
  bool ok = false;
  if (!cJSON_IsObject(root)) {
    error = "providers must be a JSON object";
  } else if (!cJSON_IsObject(entry)) {
    error = "no provider named '" + name + "'";
  } else if (!cJSON_IsString(url) || !*url->valuestring) {
    error = "provider '" + name + "' needs a \"url\"";
  } else if (headers && !cJSON_IsObject(headers)) {
    error = "provider '" + name + "': \"headers\" must be an object";
  } else {
    ok = true;
    cfg.provider = name;
    cfg.url = url->valuestring;
    double poll = 5;
    num(entry, "poll_s", poll);
    cfg.pollS = std::clamp(static_cast<int>(poll), kMinPollS, 3600);
    cfg.headers.clear();
    const cJSON* h = nullptr;
    cJSON_ArrayForEach(h, headers) {
      if (cJSON_IsString(h)) cfg.headers.emplace_back(h->string, h->valuestring);
    }
  }
  cJSON_Delete(root);
  return ok;
}

void start(const Config& cfg) {
  {
    std::lock_guard<std::mutex> g(s_mutex);
    s_cfg = cfg;
  }
  // TLS handshakes need a deep stack; mbedTLS buffers themselves go to PSRAM.
  xTaskCreatePinnedToCore(task, "feed", 12 * 1024, nullptr, 3, &s_task, 0);
}

void configure(const Config& cfg) {
  {
    std::lock_guard<std::mutex> g(s_mutex);
    s_cfg = cfg;
  }
  if (s_task) xTaskNotifyGive(s_task);
}

void pause(bool paused) {
  bool wake;
  {
    std::lock_guard<std::mutex> g(s_mutex);
    wake = s_paused && !paused;
    s_paused = paused;
  }
  if (wake && s_task) xTaskNotifyGive(s_task);
}

std::shared_ptr<const Snapshot> latest() {
  std::lock_guard<std::mutex> g(s_mutex);
  return s_latest;
}

uint32_t generation() {
  std::lock_guard<std::mutex> g(s_mutex);
  return s_generation;
}

std::string lastError() {
  std::lock_guard<std::mutex> g(s_mutex);
  return s_error;
}

Status status() {
  std::lock_guard<std::mutex> g(s_mutex);
  return s_status;
}

std::string provider() {
  std::lock_guard<std::mutex> g(s_mutex);
  return s_cfg.provider;
}

}  // namespace feed
