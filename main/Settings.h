#pragma once
#include <cstdlib>
#include <string>

#include "Feed.h"
#include "SettingsBase.h"
#include "esp_log.h"

inline constexpr char kDefaultProviders[] =
    R"({"adsb.fi":{"url":"https://opendata.adsb.fi/api/v2/lat/{lat}/lon/{lon}/dist/{nm}","poll_s":5},)"
    R"("adsb.lol":{"url":"https://api.adsb.lol/v2/point/{lat}/{lon}/{nm}","poll_s":180},)"
    R"("local":{"url":"http://readsb.local/data/aircraft.json","poll_s":1}})";

// Runtime settings, persisted as one JSON blob in NVS (mianesp settingsbase) and
// edited via GET/POST /config or the page at /. Member initialisers are the
// defaults for a fresh device. SettingsBase supports only std::string and int,
// so coordinates are text.
struct Settings : SettingsBase {
  std::string tz = "AEST-10AEDT,M10.1.0,M4.1.0/3";  // POSIX TZ, Sydney
  std::string latitude = "-33.8688";
  std::string longitude = "151.2093";
  int radiusNm = 25;  // query radius, and the radar's outer ring
  // Where aircraft come from: the active entry of the provider table (see
  // feed::parseProvider). adsb.lol allows 500 anonymous requests a day.
  std::string provider = "adsb.fi";
  std::string providers = kDefaultProviders;
  int brightness = 80;  // percent
  int ground = 1;       // parked aircraft: 0 hide, 1 group into badges, 2 show each (ui::Ground)
  int zoom = 0;         // radar range: 0 full, 1 half, 2 quarter; follows the +/- buttons
  int sweep = 1;        // rotating sweep line on the radar: 0 off, 1 on
  int units = 0;        // shown units: 0 aviation (ft, nm, kt), 1 metric (m, km, km/h)

  explicit Settings(NvsStorageManager& nvs) : SettingsBase(nvs) {
    field("tz", tz);
    field("latitude", latitude);
    field("longitude", longitude);
    field("radius_nm", radiusNm);
    field("provider", provider);
    field("providers", providers);
    field("brightness", brightness);
    field("ground", ground);
    field("zoom", zoom);
    field("sweep", sweep);
    field("units", units);
    load();  // must follow every field() registration
  }

  // Hides SettingsBase::log, which would print the provider table and so any
  // API key in its headers.
  void log() const {
    ESP_LOGI("settings", "tz=%s latitude=%s longitude=%s radius_nm=%d provider=%s brightness=%d ground=%d zoom=%d sweep=%d units=%d",
             tz.c_str(), latitude.c_str(), longitude.c_str(), radiusNm, provider.c_str(), brightness, ground, zoom,
             sweep, units);
  }

  feed::Config feedConfig() const {
    feed::Config c;
    std::string error;
    // POST /config validates the table, so this only guards against a bad NVS blob.
    if (!feed::parseProvider(providers, provider, c, error)) {
      ESP_LOGE("settings", "%s, using adsb.fi", error.c_str());
      feed::parseProvider(kDefaultProviders, "adsb.fi", c, error);
    }
    c.lat = std::strtod(latitude.c_str(), nullptr);
    c.lon = std::strtod(longitude.c_str(), nullptr);
    c.radiusNm = radiusNm < 1 ? 1 : (radiusNm > 250 ? 250 : radiusNm);  // API maximum is 250
    return c;
  }
};
