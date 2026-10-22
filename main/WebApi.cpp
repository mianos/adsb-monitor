#include "WebApi.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include "Feed.h"
#include "LogBuffer.h"
#include "WebPage.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"

namespace {

constexpr char TAG[] = "webapi";
constexpr int kMaxConfigBody = 4 * 1024;
// All settings persist as one NVS string, capped at 4000 bytes.
constexpr size_t kMaxProviders = 2500;

esp_err_t send_json(httpd_req_t* req, const JsonWrapper& json) {
  httpd_resp_set_type(req, "application/json");
  std::string out = json.ToString();
  return httpd_resp_sendstr(req, out.c_str());
}

std::string currentHostname() {
  const char* name = nullptr;
  esp_netif_t* sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (sta) esp_netif_get_hostname(sta, &name);
  return name ? name : "";
}

bool validHostname(const std::string& h) {
  if (h.empty() || h.size() > 32 || h.front() == '-' || h.back() == '-') return false;
  for (char c : h) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-')) return false;
  }
  return true;
}

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    default: return "other";
  }
}

void restartSoon() {
  vTaskDelay(pdMS_TO_TICKS(500));  // let the response flush
  esp_restart();
}

}  // namespace

esp_err_t WebApi::start() {
  esp_err_t r = WebServer::start();
  if (r != ESP_OK) return r;

  const httpd_uri_t routes[] = {
      {.uri = "/firmware", .method = HTTP_POST, .handler = firmware_post_handler, .user_ctx = this},
      {.uri = "/log", .method = HTTP_GET, .handler = log_get_handler, .user_ctx = this},
      {.uri = "/", .method = HTTP_GET, .handler = page_get_handler, .user_ctx = this},
      {.uri = "/config", .method = HTTP_GET, .handler = config_get_handler, .user_ctx = this},
      {.uri = "/config", .method = HTTP_POST, .handler = config_post_handler, .user_ctx = this},
      {.uri = "/aircraft", .method = HTTP_GET, .handler = aircraft_get_handler, .user_ctx = this},
      {.uri = "/reboot", .method = HTTP_POST, .handler = reboot_post_handler, .user_ctx = this},
  };
  for (const httpd_uri_t& uri : routes) {
    r = httpd_register_uri_handler(server, &uri);
    if (r != ESP_OK) {
      ESP_LOGE(TAG, "register %s: %s", uri.uri, esp_err_to_name(r));
      return r;
    }
  }
  return ESP_OK;
}

void WebApi::populate_healthz_fields(WebContext*, JsonWrapper& json) {
  const esp_app_desc_t* desc = esp_app_get_description();
  json.AddItem("version", std::string(desc->version));
  json.AddItem("partition", std::string(esp_ota_get_running_partition()->label));
  json.AddItem("uptime_s", static_cast<int>(esp_timer_get_time() / 1000000));
  json.AddItem("heap_free", static_cast<int>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
  json.AddItem("psram_free", static_cast<int>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
  json.AddItem("reset_reason", std::string(resetReasonName(esp_reset_reason())));
  json.AddItem("aircraft", static_cast<int>(feed::latest()->aircraft.size()));
  json.AddItem("provider", feed::provider());
  json.AddItem("feed_error", feed::lastError());
  const feed::Status fs = feed::status();
  json.AddItem("http_status", fs.httpStatus);
  json.AddItem("fetch_ms", fs.fetchMs);
  json.AddItem("last_ok_age_s",
               fs.lastOkUs ? static_cast<int>((esp_timer_get_time() - fs.lastOkUs) / 1000000) : -1);
  if (!fs.rateLimit.empty()) json.AddItem("ratelimit_limit", fs.rateLimit);
  if (fs.rateRemaining >= 0) json.AddItem("ratelimit_remaining", fs.rateRemaining);
  if (fs.retryAfterS >= 0) json.AddItem("retry_after_s", fs.retryAfterS);
  // Lowest free stack ever seen on the main loop's task, in bytes.
  if (TaskHandle_t main = xTaskGetHandle("main")) {
    json.AddItem("main_stack_min_free", static_cast<int>(uxTaskGetStackHighWaterMark(main)));
  }
}

// Raw .bin body streamed into the inactive OTA slot, which becomes the next boot
// partition, then reboot.
esp_err_t WebApi::firmware_post_handler(httpd_req_t* req) {
  const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
  if (!target) return sendJsonError(req, 500, "no OTA partition available");
  if (req->content_len <= 0) return sendJsonError(req, 400, "empty body (POST the .bin)");
  if (static_cast<size_t>(req->content_len) > target->size)
    return sendJsonError(req, 413, "image larger than the OTA partition");

  ESP_LOGW(TAG, "OTA: %d bytes -> %s", req->content_len, target->label);

  esp_ota_handle_t handle = 0;
  esp_err_t err = esp_ota_begin(target, req->content_len, &handle);
  if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

  char buf[1024];
  int remaining = req->content_len;
  while (remaining > 0) {
    const int got = httpd_req_recv(req, buf, std::min<int>(remaining, sizeof buf));
    if (got == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (got <= 0) {
      esp_ota_abort(handle);
      return sendJsonError(req, 400, "upload aborted");
    }
    err = esp_ota_write(handle, buf, got);
    if (err != ESP_OK) {
      esp_ota_abort(handle);
      return sendJsonError(req, 500, esp_err_to_name(err));
    }
    remaining -= got;
  }

  err = esp_ota_end(handle);
  if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));
  err = esp_ota_set_boot_partition(target);
  if (err != ESP_OK) return sendJsonError(req, 500, esp_err_to_name(err));

  JsonWrapper resp;
  resp.AddItem("status", std::string("ok, rebooting"));
  resp.AddItem("partition", std::string(target->label));
  esp_err_t r = send_json(req, resp);
  ESP_LOGW(TAG, "OTA complete, rebooting into %s", target->label);
  vTaskDelay(pdMS_TO_TICKS(500));  // let the response flush
  esp_restart();
  return r;
}

esp_err_t WebApi::log_get_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/plain");
  char query[16] = {};
  httpd_req_get_url_query_str(req, query, sizeof query);
  std::string log = std::string(query).starts_with("prev") ? LogBuffer::previousBoot() : LogBuffer::snapshot();
  return httpd_resp_send(req, log.data(), log.size());
}

esp_err_t WebApi::page_get_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, kSettingsPage, HTTPD_RESP_USE_STRLEN);
}

esp_err_t WebApi::config_get_handler(httpd_req_t* req) {
  auto* self = static_cast<WebApi*>(req->user_ctx);
  JsonWrapper resp = self->settings_.toJson();
  resp.AddItem("hostname", currentHostname());
  return send_json(req, resp);
}

// Applies any subset of the settings keys and persists. Everything takes effect
// via onChange hooks in main.cpp, except a hostname change, which restarts (the
// hostname is only sent to DHCP when the lease is taken).
esp_err_t WebApi::config_post_handler(httpd_req_t* req) {
  auto* self = static_cast<WebApi*>(req->user_ctx);
  Settings& settings = self->settings_;
  if (req->content_len <= 0 || req->content_len > kMaxConfigBody) {
    return sendJsonError(req, 400, "body must be 1-4096 bytes of JSON");
  }
  std::string body(req->content_len, '\0');
  for (int got = 0; got < req->content_len;) {
    const int n = httpd_req_recv(req, body.data() + got, req->content_len - got);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
    if (n <= 0) return sendJsonError(req, 400, "upload aborted");
    got += n;
  }
  JsonWrapper json = JsonWrapper::Parse(body);
  if (json.Empty()) return sendJsonError(req, 400, "invalid JSON");

  std::string hostname;
  const bool hostnameChanged = json.GetField("hostname", hostname) && hostname != currentHostname();
  if (hostnameChanged && !validHostname(hostname)) {
    return sendJsonError(req, 400, "hostname: 1-32 letters, digits or '-', not starting/ending with '-'");
  }
  // Check the provider pair as it will be after this update, so a bad table or
  // unknown name is refused rather than leaving the radar without a feed.
  std::string provider = settings.provider, providers = settings.providers;
  const bool providerSent = json.GetField("provider", provider);
  const bool providersSent = json.GetField("providers", providers);
  if (providers.size() > kMaxProviders) {
    return sendJsonError(req, 400, "providers: at most 2500 bytes");
  }
  feed::Config probe;
  std::string error;
  if ((providerSent || providersSent) && !feed::parseProvider(providers, provider, probe, error)) {
    return sendJsonError(req, 400, error);
  }

  if (hostnameChanged) self->webContext->wifiManager->configSetHostName(hostname);

  settings.loadFromJson(json);
  settings.save();
  settings.log();

  JsonWrapper resp = settings.toJson();
  resp.AddItem("hostname", hostnameChanged ? hostname : currentHostname());
  resp.AddItem("rebooting", hostnameChanged);
  esp_err_t r = send_json(req, resp);
  if (hostnameChanged) {
    ESP_LOGW(TAG, "hostname changed, restarting");
    restartSoon();
  }
  return r;
}

// JsonWrapper has no arrays, so this one is built with cJSON directly.
esp_err_t WebApi::aircraft_get_handler(httpd_req_t* req) {
  auto snap = feed::latest();
  cJSON* root = cJSON_CreateObject();
  cJSON_AddNumberToObject(root, "age_s",
                          snap->fetchedUs ? (esp_timer_get_time() - snap->fetchedUs) / 1000000.0 : -1);
  cJSON* list = cJSON_AddArrayToObject(root, "aircraft");
  for (const Aircraft& a : snap->aircraft) {
    cJSON* o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "hex", a.hex.c_str());
    cJSON_AddStringToObject(o, "flight", a.flight.c_str());
    cJSON_AddStringToObject(o, "reg", a.reg.c_str());
    cJSON_AddStringToObject(o, "type", a.type.c_str());
    if (a.onGround) {
      cJSON_AddStringToObject(o, "alt_ft", "ground");
    } else if (a.hasAlt) {
      cJSON_AddNumberToObject(o, "alt_ft", a.altFt);
    }
    if (a.gsKt >= 0) cJSON_AddNumberToObject(o, "gs_kt", a.gsKt);
    if (a.hasTrack) cJSON_AddNumberToObject(o, "track", a.track);
    cJSON_AddNumberToObject(o, "dist_nm", std::round(a.distNm * 10) / 10);
    cJSON_AddNumberToObject(o, "bearing", std::round(a.bearing));
    cJSON_AddItemToArray(list, o);
  }
  char* out = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if (!out) return sendJsonError(req, 500, "out of memory");
  httpd_resp_set_type(req, "application/json");
  esp_err_t r = httpd_resp_sendstr(req, out);
  cJSON_free(out);
  return r;
}

esp_err_t WebApi::reboot_post_handler(httpd_req_t* req) {
  JsonWrapper resp;
  resp.AddItem("status", std::string("rebooting"));
  esp_err_t r = send_json(req, resp);
  ESP_LOGW(TAG, "restart requested over HTTP");
  restartSoon();
  return r;
}
