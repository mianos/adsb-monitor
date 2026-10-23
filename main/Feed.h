#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// One aircraft from the API, plus range and bearing from home.
struct Aircraft {
  std::string hex;     // ICAO 24-bit address, the stable key
  std::string flight;  // callsign, trimmed; may be empty
  std::string reg;
  std::string type;    // ICAO type designator, e.g. B738
  std::string squawk;
  bool onGround = false;
  bool hasAlt = false;
  int altFt = 0;  // barometric, falling back to geometric
  bool hasTrack = false;
  float track = 0;  // degrees true
  float gsKt = -1;  // ground speed, -1 unknown
  int vrateFpm = 0;
  bool emergency = false;
  bool rotorcraft = false;  // ADS-B emitter category A7
  double lat = 0, lon = 0;
  float posAgeS = 0;  // how old the position already was when fetched
  float distNm = 0;
  float bearing = 0;  // degrees true from home

  const std::string& label() const { return !flight.empty() ? flight : !reg.empty() ? reg : hex; }
};

struct Snapshot {
  std::vector<Aircraft> aircraft;  // nearest first
  int64_t fetchedUs = 0;           // esp_timer time of the fetch
};

// Polls the ADS-B provider for aircraft around home on its own task. The latest
// good snapshot is shared read-only; a failed poll keeps the previous one.
namespace feed {

struct Config {
  std::string provider;  // name of the entry in the provider table
  std::string url;       // template: {lat}, {lon} and {nm} are filled in
  std::vector<std::pair<std::string, std::string>> headers;
  double lat = 0, lon = 0;
  int radiusNm = 25;
  int pollS = 5;
};

// Fills provider, url, pollS and headers from the named entry of a provider
// table (a JSON object of {"name": {"url", "poll_s"?, "headers"?}}). On failure
// leaves cfg alone and says why in error.
bool parseProvider(const std::string& table, const std::string& name, Config& cfg, std::string& error);

void start(const Config& cfg);
void configure(const Config& cfg);  // takes effect on the next poll, which runs now
void pause(bool paused);            // while offline

std::shared_ptr<const Snapshot> latest();
uint32_t generation();  // bumps on every successful poll
std::string lastError();  // empty after a good poll
std::string provider();   // name of the provider being polled

// What the last request saw, for spotting throttling remotely.
struct Status {
  int httpStatus = 0;     // 0 if no response
  int fetchMs = -1;       // how long the request took
  int64_t lastOkUs = 0;   // esp_timer time of the last good poll, 0 if none
  std::string rateLimit;  // x-ratelimit-limit as sent, e.g. "60;w=60, 500;w=86400"
  int rateRemaining = -1; // x-ratelimit-remaining, -1 if not sent
  int retryAfterS = -1;   // Retry-After in seconds, -1 if not sent
};
Status status();

}  // namespace feed
