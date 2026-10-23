#include "Ui.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "Board.h"
#include "Display.h"
#include "Feed.h"
#include "Geo.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

namespace ui {
namespace {

constexpr int kScope = board::kHeight;  // square scope, full height
constexpr int kScopeR = kScope / 2 - 10;
// The case shows the scope through a round window; the square's corners are
// hidden, so everything on the scope is kept inside this radius.
constexpr int kVisibleR = kScope / 2 - 2;
constexpr int kPanelX = kScope;
constexpr int kPanelW = board::kWidth - kScope;
// Margin inside the panel's rectangular window, so its frame doesn't cover text.
constexpr int kInset = 4;
constexpr int kInnerW = kPanelW - 2 * kInset;
constexpr int kControlsH = 96;
constexpr int kListH = board::kHeight - 2 * kInset - kControlsH;
constexpr int kRowH = 24;
constexpr int kMaxRows = 30;
constexpr size_t kTrailLen = 40;
constexpr int kPickRadiusPx = 22;
constexpr float kZoom[] = {1.0f, 0.5f, 0.25f};
constexpr int kZoomLevels = sizeof kZoom / sizeof kZoom[0];
constexpr float kClusterPx = 18;
// Between polls an aircraft is carried on along its track at its ground speed,
// but not for ever if the feed goes quiet.
constexpr float kReckonMaxS = 30;
// A poll's correction to a shown position is spread over this long.
constexpr int64_t kEaseUs = 2000000;
constexpr float kEaseMaxNm = 2;  // bigger corrections just jump
// The sweep is decoration only: positions come from the feed, not the beam.
constexpr int64_t kSweepPeriodUs = 4000000;
constexpr uint32_t kSweepFrameMs = 66;
// A real fading wedge means blending a large area every frame, so the
// afterglow is a few fainter lines trailing the beam instead.
constexpr lv_opa_t kSweepOpa[] = {LV_OPA_50, LV_OPA_30, LV_OPA_20, LV_OPA_10};
constexpr float kSweepGap = 3 * 3.14159265f / 180;
constexpr float kSweepTrail = kSweepGap * (sizeof kSweepOpa - 1);

const lv_color_t kBg = lv_color_hex(0x000000);
const lv_color_t kRing = lv_color_hex(0x1f4d2b);
const lv_color_t kRingText = lv_color_hex(0x3f8f55);
const lv_color_t kText = lv_color_hex(0xe6e6e6);
const lv_color_t kDim = lv_color_hex(0x8a8a8a);
const lv_color_t kPanelBg = lv_color_hex(0x101418);
const lv_color_t kDivider = lv_color_hex(0x263038);
const lv_color_t kButton = lv_color_hex(0x263038);
const lv_color_t kButtonPressed = lv_color_hex(0x3a4a56);
const lv_color_t kSelect = lv_color_hex(0xffffff);
const lv_color_t kEmergency = lv_color_hex(0xff3030);
const lv_color_t kSweep = kRingText;
const lv_font_t* const kRadarFont = &lv_font_montserrat_10;

struct Point {
  float eastNm, northNm;
};

struct Cluster {
  float x, y;  // scope coordinates of the first member
  int count;
};

// Owned by the LVGL task (draw and event callbacks, the tick timer).
struct State {
  double homeLat = 0, homeLon = 0;
  int radiusNm = 25;
  Ground ground = Ground::Group;
  bool configOpen = false;
  int zoom = 0;
  bool metric = false;
  std::shared_ptr<const Snapshot> snap = std::make_shared<Snapshot>();
  uint32_t generation = 0;
  std::string selected;  // hex
  std::unordered_map<std::string, std::deque<Point>> trails;
  // Where each aircraft was shown when its latest fix arrived, relative to that
  // fix, so the marker eases over rather than jumps.
  struct Ease {
    float eastNm, northNm;
    int64_t sinceUs;
  };
  std::unordered_map<std::string, Ease> ease;
  std::unordered_map<std::string, int> labelSpot;  // last spot used, so labels don't flip about
  // With the sweep on, each aircraft stays where the beam last painted it, as on a real scope.
  std::unordered_map<std::string, Point> painted;
  std::vector<std::string> rowHex;
  Net net = Net::Connecting;
  std::string message;
  std::string error;
  int lastMinute = -1;
} s;

// Written from other tasks, picked up by the tick timer.
std::mutex s_inMutex;
Net s_inNet = Net::Connecting;
std::string s_inMessage;
bool s_inHomeDirty = false;
double s_inLat = 0, s_inLon = 0;
int s_inRadius = 25;
Ground s_inGround = Ground::Group;
int s_inZoom = 0;
bool s_inSweep = true;
bool s_inMetric = false;
void (*s_unitsChosen)(bool) = nullptr;
void (*s_groundChosen)(Ground) = nullptr;
void (*s_zoomChosen)(int) = nullptr;
void (*s_sweepChosen)(bool) = nullptr;

bool s_sweepOn = false;
float s_sweepAngle = 0;  // of the beam, radians clockwise from north; what drawScope draws

// List rows are created once and refilled, so a poll arriving mid-scroll
// doesn't delete the row under the finger.
struct Row {
  lv_obj_t* obj;
  std::string name, alt, dist;
  lv_color_t color;
};

lv_obj_t* s_scope;
lv_obj_t* s_scopeMsg;
lv_obj_t* s_clock;
lv_obj_t* s_status;
lv_obj_t* s_link;
lv_obj_t* s_range;
lv_obj_t* s_zoomOut;
lv_obj_t* s_zoomIn;
lv_obj_t* s_configBtn;
lv_obj_t* s_config;
constexpr Ground kGroundModes[] = {Ground::Group, Ground::Show, Ground::Hide};
constexpr const char* kGroundNames[] = {"Group", "Show each", "Hide"};
lv_obj_t* s_groundOpts[3];
lv_obj_t* s_sweepBtn;
lv_obj_t* s_metricBtn;
lv_obj_t* s_list;
Row s_rows[kMaxRows];
lv_obj_t* s_detail;
lv_obj_t* s_detailTitle;
lv_obj_t* s_detailBody;

lv_color_t altColor(const Aircraft& a) {
  if (a.emergency) return kEmergency;
  if (!a.hasAlt) return kDim;
  if (a.onGround) return lv_color_hex(0x9a9a9a);
  if (a.altFt < 2000) return lv_color_hex(0xff9a2e);
  if (a.altFt < 10000) return lv_color_hex(0xffe14d);
  if (a.altFt < 20000) return lv_color_hex(0x6be36b);
  if (a.altFt < 30000) return lv_color_hex(0x4dd8ff);
  return lv_color_hex(0xc58cff);
}

float viewNm() { return s.radiusNm * kZoom[s.zoom]; }
// Farthest an aircraft can be and still show on the radar (the edge of the window).
float radarLimitNm() { return viewNm() * kVisibleR / kScopeR; }

lv_point_precise_t pt(float x, float y) {
  return {static_cast<lv_value_precise_t>(std::lround(x)), static_cast<lv_value_precise_t>(std::lround(y))};
}

// Whether the aircraft gets its own marker, label, trail and tap target. A parked
// aircraft picked from the list is pulled out of its badge.
bool drawnSingly(const Aircraft& a) {
  return !a.onGround || s.ground == Ground::Show || (s.ground == Ground::Group && a.hex == s.selected);
}

bool overlaps(const lv_area_t& a, const lv_area_t& b) {
  return a.x1 <= b.x2 && b.x1 <= a.x2 && a.y1 <= b.y2 && b.y1 <= a.y2;
}

bool inWindow(const lv_area_t& a, int cx, int cy) {
  const int r2 = (kVisibleR - 2) * (kVisibleR - 2);
  for (int x : {a.x1, a.x2}) {
    for (int y : {a.y1, a.y2}) {
      if ((x - cx) * (x - cx) + (y - cy) * (y - cy) > r2) return false;
    }
  }
  return true;
}

// Setting a label repaints it even when nothing changed, so check first.
void setText(lv_obj_t* label, const char* text) {
  if (strcmp(lv_label_get_text(label), text) != 0) lv_label_set_text(label, text);
}

void setTextColor(lv_obj_t* label, lv_color_t color) {
  if (!lv_color_eq(lv_obj_get_style_text_color(label, 0), color)) lv_obj_set_style_text_color(label, color, 0);
}

// Distances are kept in nm, as the feed and the radius setting use them.
float distOut(float nm) { return s.metric ? nm * 1.852f : nm; }
const char* distUnit() { return s.metric ? " km" : " nm"; }

void rangeText(char* buf, size_t len, float nm, const char* suffix) {
  const float d = distOut(nm);
  snprintf(buf, len, d < 10 ? "%.1f%s" : "%.0f%s", d, suffix);
}

const Aircraft* findSelected();
void renderBackground();

// Screen position relative to the scope centre.
void toScope(float eastNm, float northNm, float& x, float& y) {
  const float scale = kScopeR / viewNm();
  x = eastNm * scale;
  y = -northNm * scale;
}

void position(const Aircraft& a, float& eastNm, float& northNm) {
  eastNm = a.distNm * std::sin(a.bearing * geo::kDeg);
  northNm = a.distNm * std::cos(a.bearing * geo::kDeg);
}

// The fix carried forward to now along the track at the ground speed.
void reckoned(const Aircraft& a, int64_t nowUs, float& eastNm, float& northNm) {
  position(a, eastNm, northNm);
  if (a.onGround || a.gsKt <= 0 || !a.hasTrack) return;
  const float ageS = std::clamp((nowUs - s.snap->fetchedUs) / 1e6f, 0.0f, kReckonMaxS);
  const float nm = a.gsKt * ageS / 3600;
  eastNm += nm * std::sin(a.track * geo::kDeg);
  northNm += nm * std::cos(a.track * geo::kDeg);
}

// Where the aircraft is drawn: reckoned, plus what is left of the correction the
// latest fix made to the position shown before it.
void shownPosition(const Aircraft& a, int64_t nowUs, float& eastNm, float& northNm) {
  reckoned(a, nowUs, eastNm, northNm);
  auto it = s.ease.find(a.hex);
  if (it == s.ease.end()) return;
  const float left = 1 - static_cast<float>(nowUs - it->second.sinceUs) / kEaseUs;
  if (left <= 0) return;
  eastNm += it->second.eastNm * left;
  northNm += it->second.northNm * left;
}

// False for an aircraft the sweep hasn't reached yet.
bool drawnPosition(const Aircraft& a, int64_t nowUs, float& eastNm, float& northNm) {
  if (s_sweepOn) {
    auto it = s.painted.find(a.hex);
    if (it == s.painted.end()) return false;
    eastNm = it->second.eastNm;
    northNm = it->second.northNm;
    return true;
  }
  shownPosition(a, nowUs, eastNm, northNm);
  return true;
}

bool unpainted(const Aircraft& a) { return s_sweepOn && !s.painted.count(a.hex); }

// Called with the new snapshot in place and shown holding where each aircraft
// was drawn a moment before it arrived.
void startEasing(const std::unordered_map<std::string, Point>& shown, int64_t nowUs) {
  std::unordered_map<std::string, State::Ease> next;
  for (const Aircraft& a : s.snap->aircraft) {
    auto it = shown.find(a.hex);
    if (it == shown.end()) continue;
    float east, north;
    reckoned(a, nowUs, east, north);
    const float dEast = it->second.eastNm - east, dNorth = it->second.northNm - north;
    if (std::hypot(dEast, dNorth) > kEaseMaxNm) continue;
    next[a.hex] = {dEast, dNorth, nowUs};
  }
  s.ease = std::move(next);
}

std::string altText(const Aircraft& a) {
  if (!a.hasAlt) return "";
  if (a.onGround) return "GND";
  char buf[12];
  if (s.metric) {
    snprintf(buf, sizeof buf, "%ld", std::lround(a.altFt * 0.3048f / 10) * 10);
  } else if (a.altFt >= 18000) {
    snprintf(buf, sizeof buf, "FL%03d", (a.altFt + 50) / 100);
  } else {
    snprintf(buf, sizeof buf, "%d", (a.altFt + 50) / 100 * 100);
  }
  return buf;
}

// A climb or descent worth noting; level flight shows nothing.
const char* climbSymbol(const Aircraft& a) {
  if (a.onGround || !a.hasAlt) return "";
  return a.vrateFpm > 300 ? " " LV_SYMBOL_UP : (a.vrateFpm < -300 ? " " LV_SYMBOL_DOWN : "");
}

void drawLabel(lv_layer_t* layer, const char* text, const lv_font_t* font, lv_color_t color, int x, int y,
               int w = 120) {
  lv_draw_label_dsc_t d;
  lv_draw_label_dsc_init(&d);
  d.text = text;
  d.text_local = 1;
  d.font = font;
  d.color = color;
  lv_area_t area = {x, y, x + w, y + lv_font_get_line_height(font)};
  lv_draw_label(layer, &d, &area);
}

void drawRing(lv_layer_t* layer, float x, float y, int radius, int width, lv_color_t color) {
  lv_draw_arc_dsc_t d;
  lv_draw_arc_dsc_init(&d);
  d.color = color;
  d.width = width;
  d.radius = radius;
  d.start_angle = 0;
  d.end_angle = 360;
  d.center = {static_cast<int32_t>(std::lround(x)), static_cast<int32_t>(std::lround(y))};
  lv_draw_arc(layer, &d);
}

// A lone parked aircraft is a small dot; a group is a grey badge with the count.
// The area it covers, relative to the scope centre, so labels keep clear of it.
lv_area_t badgeArea(float x, float y, int count) {
  const int ix = std::lround(x), iy = std::lround(y);
  if (count == 1) return {ix - 3, iy - 3, ix + 3, iy + 3};
  char text[8];
  snprintf(text, sizeof text, "%d", count);
  lv_point_t size;
  lv_text_get_size(&size, text, kRadarFont, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  const int w = std::max<int>(size.x + 8, size.y + 2), h = size.y + 2;
  return {ix - w / 2, iy - h / 2, ix - w / 2 + w - 1, iy - h / 2 + h - 1};
}

void drawBadge(lv_layer_t* layer, const lv_area_t& area, int count) {
  if (count == 1) {
    drawRing(layer, (area.x1 + area.x2) / 2.0f, (area.y1 + area.y2) / 2.0f, 2, 3, lv_color_hex(0x9a9a9a));
    return;
  }
  char text[8];
  snprintf(text, sizeof text, "%d", count);
  const int h = lv_area_get_height(&area);
  lv_draw_rect_dsc_t bg;
  lv_draw_rect_dsc_init(&bg);
  bg.bg_color = lv_color_hex(0x5a5a5a);
  bg.bg_opa = LV_OPA_COVER;
  bg.radius = h / 2;
  lv_draw_rect(layer, &bg, &area);
  lv_draw_label_dsc_t d;
  lv_draw_label_dsc_init(&d);
  d.text = text;
  d.text_local = 1;
  d.font = kRadarFont;
  d.color = lv_color_hex(0xf0f0f0);
  d.align = LV_TEXT_ALIGN_CENTER;
  lv_area_t textArea = area;
  textArea.y1 += 1;
  lv_draw_label(layer, &d, &textArea);
}

// Everything placed on the scope, relative to its centre. It is worked out by
// updateScope, which repaints only the parts that differ from the last layout;
// LVGL renders those in 40-line strips and runs drawScope for each, and each
// strip only draws what touches it.
struct Segment {
  lv_point_precise_t p0, p1;
  lv_color_t color;
};
struct Badge {
  lv_area_t area;
  int count;
};
struct Marker {
  int x, y;  // whole pixels, so sub-pixel drift between polls repaints nothing
  float track;  // radians
  bool arrow, ground, selected, heli;
  lv_color_t color;
};
struct Callout {
  lv_area_t area;
  std::string text;
  lv_color_t color;
};
struct Layout {
  float viewNm = 0;
  std::vector<Segment> trails;
  std::vector<Badge> badges;
  std::vector<Marker> markers;  // farthest first, so near aircraft draw on top
  std::vector<Callout> callouts;
} s_layout;

void computeLayout() {
  Layout& l = s_layout;
  l.viewNm = viewNm();
  l.trails.clear();
  l.badges.clear();
  l.markers.clear();
  l.callouts.clear();
  const float limit = radarLimitNm();
  const int64_t now = esp_timer_get_time();

  for (const Aircraft& a : s.snap->aircraft) {
    if (a.distNm > limit || !drawnSingly(a) || unpainted(a)) continue;  // no trail without its marker
    auto it = s.trails.find(a.hex);
    if (it == s.trails.end()) continue;
    const auto& pts = it->second;
    for (size_t i = 1; i < pts.size(); ++i) {
      float x0, y0, x1, y1;
      toScope(pts[i - 1].eastNm, pts[i - 1].northNm, x0, y0);
      toScope(pts[i].eastNm, pts[i].northNm, x1, y1);
      if (std::hypot(x0, y0) > kVisibleR || std::hypot(x1, y1) > kVisibleR) continue;
      l.trails.push_back({pt(x0, y0), pt(x1, y1), altColor(a)});
    }
  }

  // Parked aircraft pile up at airports, so in Group mode they collapse into
  // count badges and take no labels.
  std::vector<Cluster> clusters;
  if (s.ground == Ground::Group) {
    for (const Aircraft& a : s.snap->aircraft) {
      if (drawnSingly(a) || a.distNm > limit) continue;
      float east, north, x, y;
      position(a, east, north);
      toScope(east, north, x, y);
      auto near = std::find_if(clusters.begin(), clusters.end(),
                               [&](const Cluster& c) { return std::hypot(c.x - x, c.y - y) < kClusterPx; });
      if (near == clusters.end()) {
        clusters.push_back({x, y, 1});
      } else {
        ++near->count;
      }
    }
  }
  std::vector<lv_area_t> taken;
  for (const Cluster& c : clusters) {
    l.badges.push_back({badgeArea(c.x, c.y, c.count), c.count});
    taken.push_back(l.badges.back().area);
  }

  for (auto it = s.snap->aircraft.rbegin(); it != s.snap->aircraft.rend(); ++it) {
    const Aircraft& a = *it;
    if (a.distNm > limit || !drawnSingly(a)) continue;
    float east, north, x, y;
    if (!drawnPosition(a, now, east, north)) continue;
    toScope(east, north, x, y);
    if (std::hypot(x, y) > kVisibleR) continue;  // reckoned out of the window
    l.markers.push_back({static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
                         static_cast<float>(a.track * geo::kDeg), a.hasTrack && !a.onGround, a.onGround,
                         a.hex == s.selected, a.rotorcraft && !a.onGround, altColor(a)});
  }

  // Callouts: the selected aircraft, then nearest first, each in the first of four
  // spots around its marker that is clear of labels already placed, trying the
  // spot it had last time first. Aircraft that find no spot stay unlabelled
  // (still tappable).
  std::vector<const Aircraft*> order;
  if (const Aircraft* sel = findSelected()) order.push_back(sel);
  for (const Aircraft& a : s.snap->aircraft) {
    if (a.hex != s.selected) order.push_back(&a);
  }
  std::unordered_map<std::string, int> spotUsed;
  char buf[48];
  for (const Aircraft* a : order) {
    if (a->distNm > limit || !drawnSingly(*a)) continue;
    float east, north, x, y;
    if (!drawnPosition(*a, now, east, north)) continue;
    toScope(east, north, x, y);
    const int px = std::lround(x), py = std::lround(y);
    // Just the callsign; altitude is the marker's colour and is in the list.
    snprintf(buf, sizeof buf, "%s", a->label().c_str());
    lv_point_t size;
    lv_text_get_size(&size, buf, kRadarFont, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    const int w = size.x + 2, h = size.y;
    const lv_point_t spots[] = {{px + 9, py - h / 2}, {px - 9 - w, py - h / 2}, {px - w / 2, py - 10 - h},
                                {px - w / 2, py + 10}};
    auto last = s.labelSpot.find(a->hex);
    const int first = last == s.labelSpot.end() ? 0 : last->second;
    for (int n = -1; n < 4; ++n) {
      const int i = n < 0 ? first : n;
      if (n >= 0 && i == first) continue;
      const lv_point_t& sp = spots[i];
      const lv_area_t area = {sp.x, sp.y, sp.x + w - 1, sp.y + h - 1};
      if (!inWindow(area, 0, 0) ||
          std::any_of(taken.begin(), taken.end(), [&](const lv_area_t& t) { return overlaps(t, area); }))
        continue;
      l.callouts.push_back({area, buf, a->hex == s.selected ? kSelect : altColor(*a)});
      taken.push_back(area);
      spotUsed[a->hex] = i;
      break;
    }
  }
  s.labelSpot = std::move(spotUsed);
}

lv_area_t offset(const lv_area_t& a, int dx, int dy) { return {a.x1 + dx, a.y1 + dy, a.x2 + dx, a.y2 + dy}; }

// Each drawn thing as the area it covers and a hash of how it is drawn, so two
// layouts can be compared without knowing which aircraft is which.
struct Piece {
  uint64_t key;
  lv_area_t area;
};

uint64_t mix(uint64_t h, int32_t v) { return (h ^ static_cast<uint32_t>(v)) * 0x100000001b3ULL; }
uint64_t mix(uint64_t h, const lv_area_t& a) { return mix(mix(mix(mix(h, a.x1), a.y1), a.x2), a.y2); }
uint64_t mix(uint64_t h, lv_color_t c) { return mix(h, c.red << 16 | c.green << 8 | c.blue); }

std::vector<Piece> pieces(const Layout& l) {
  std::vector<Piece> out;
  out.reserve(l.trails.size() + l.badges.size() + l.markers.size() + l.callouts.size());
  for (const Segment& t : l.trails) {
    // Anti-aliasing reaches a pixel past the end points.
    const lv_area_t a = {static_cast<int32_t>(std::min(t.p0.x, t.p1.x)) - 1,
                         static_cast<int32_t>(std::min(t.p0.y, t.p1.y)) - 1,
                         static_cast<int32_t>(std::max(t.p0.x, t.p1.x)) + 1,
                         static_cast<int32_t>(std::max(t.p0.y, t.p1.y)) + 1};
    out.push_back({mix(mix(1, a), t.color), a});
  }
  for (const Badge& b : l.badges) out.push_back({mix(mix(2, b.area), b.count), b.area});
  for (const Marker& m : l.markers) {
    const int reach = m.selected ? 15 : 10;  // the selection ring is the widest part
    const lv_area_t a = {m.x - reach, m.y - reach, m.x + reach, m.y + reach};
    uint64_t h = mix(mix(mix(3, a), static_cast<int32_t>(m.track * 1000)), m.color);
    out.push_back({mix(h, m.heli << 3 | m.arrow << 2 | m.ground << 1 | m.selected), a});
  }
  for (const Callout& c : l.callouts) {
    uint64_t h = mix(mix(4, c.area), c.color);
    for (char ch : c.text) h = mix(h, ch);
    out.push_back({h, c.area});
  }
  return out;
}

// LVGL keeps 32 invalid areas and falls back to the whole screen past that, so
// the patches are merged down to a few first. Absolute coordinates.
void invalidatePatches(std::vector<lv_area_t> patches) {
  // Each part costs a fixed amount however small, so fewer, bigger patches win.
  constexpr size_t kMaxPatches = 6;
  auto size = [](const lv_area_t& a) { return (a.x2 - a.x1 + 1) * (a.y2 - a.y1 + 1); };
  std::vector<lv_area_t> merged;
  for (const lv_area_t& p : patches) {
    size_t best = 0;
    int32_t bestCost = INT32_MAX;
    lv_area_t bestJoin{};
    for (size_t i = 0; i < merged.size(); ++i) {
      const lv_area_t& m = merged[i];
      const lv_area_t join = {std::min(m.x1, p.x1), std::min(m.y1, p.y1), std::max(m.x2, p.x2), std::max(m.y2, p.y2)};
      const int32_t cost = size(join) - size(m) - size(p);  // pixels the join adds beyond the two
      if (cost < bestCost) {
        bestCost = cost;
        best = i;
        bestJoin = join;
      }
    }
    if (merged.empty() || (bestCost > 0 && merged.size() < kMaxPatches)) {
      merged.push_back(p);
    } else {
      merged[best] = bestJoin;
    }
  }
  for (const lv_area_t& m : merged) lv_obj_invalidate_area(s_scope, &m);
}

// Lays the scope out afresh and repaints only what differs from the last layout.
// A range change moves everything, rings and labels included, so that repaints
// the lot.
std::vector<Piece> s_pieces;  // of s_layout, sorted by key

void updateScope() {
  std::vector<Piece> was = std::move(s_pieces);
  const float wasNm = s_layout.viewNm;
  computeLayout();
  std::vector<Piece>& now = s_pieces;
  now = pieces(s_layout);
  std::sort(now.begin(), now.end(), [](const Piece& a, const Piece& b) { return a.key < b.key; });
  if (s_layout.viewNm != wasNm) {
    renderBackground();
    lv_obj_invalidate(s_scope);
    return;
  }
  lv_area_t box;
  lv_obj_get_coords(s_scope, &box);
  const int cx = box.x1 + kScope / 2, cy = box.y1 + kScope / 2;
  std::vector<lv_area_t> patches;
  size_t i = 0, j = 0;
  while (i < was.size() || j < now.size()) {
    if (j == now.size() || (i < was.size() && was[i].key < now[j].key)) {
      patches.push_back(offset(was[i++].area, cx, cy));
    } else if (i == was.size() || now[j].key < was[i].key) {
      patches.push_back(offset(now[j++].area, cx, cy));
    } else {
      ++i;
      ++j;
    }
  }
  if (!patches.empty()) invalidatePatches(std::move(patches));
}

// The rings, cross, labels and home mark change only with the range, and each
// LVGL arc or label costs the same few hundred microseconds however little of
// it a strip shows, so they are rendered once into an image that the strips
// copy from. In PSRAM: LVGL's own pool is far too small for it.
lv_draw_buf_t s_bgBuf;
lv_obj_t* s_bgCanvas;
float s_bgNm = -1;

void renderBackground() {
  if (s_bgNm == viewNm()) return;
  s_bgNm = viewNm();
  lv_canvas_fill_bg(s_bgCanvas, kBg, LV_OPA_COVER);
  lv_layer_t layer;
  lv_canvas_init_layer(s_bgCanvas, &layer);
  const int cx = kScope / 2, cy = kScope / 2;

  lv_draw_arc_dsc_t ring;
  lv_draw_arc_dsc_init(&ring);
  ring.color = kRing;
  ring.width = 1;
  ring.start_angle = 0;
  ring.end_angle = 360;
  ring.center = {cx, cy};
  char buf[48];
  for (int i = 1; i <= 3; ++i) {
    ring.radius = kScopeR * i / 3;
    lv_draw_arc(&layer, &ring);
    rangeText(buf, sizeof buf, viewNm() * i / 3, "");
    drawLabel(&layer, buf, kRadarFont, kRingText, cx + 3, cy - ring.radius + 1, 40);
  }
  lv_draw_line_dsc_t cross;
  lv_draw_line_dsc_init(&cross);
  cross.color = kRing;
  cross.width = 1;
  cross.p1 = {static_cast<lv_value_precise_t>(cx - kScopeR), static_cast<lv_value_precise_t>(cy)};
  cross.p2 = {static_cast<lv_value_precise_t>(cx + kScopeR), static_cast<lv_value_precise_t>(cy)};
  lv_draw_line(&layer, &cross);
  cross.p1 = {static_cast<lv_value_precise_t>(cx), static_cast<lv_value_precise_t>(cy - kScopeR)};
  cross.p2 = {static_cast<lv_value_precise_t>(cx), static_cast<lv_value_precise_t>(cy + kScopeR)};
  lv_draw_line(&layer, &cross);
  // Inside the outer ring, opposite its distance label, so the window's edge can't cut it.
  drawLabel(&layer, "N", &lv_font_montserrat_12, kRingText, cx - 14, cy - kScopeR + 1, 20);
  drawRing(&layer, cx, cy, 3, 2, kText);  // home
  lv_canvas_finish_layer(s_bgCanvas, &layer);
}

void drawScope(lv_event_t* e) {
  lv_layer_t* layer = lv_event_get_layer(e);
  lv_area_t box;
  lv_obj_get_coords(s_scope, &box);
  const int cx = box.x1 + kScope / 2, cy = box.y1 + kScope / 2;

  lv_draw_image_dsc_t bg;
  lv_draw_image_dsc_init(&bg);
  bg.src = &s_bgBuf;
  lv_draw_image(layer, &bg, &box);

  // The strip being rendered; anything outside it would only be queued and clipped away.
  const lv_area_t strip = offset(layer->_clip_area, -cx, -cy);
  auto inStrip = [&](const lv_area_t& a) { return overlaps(a, strip); };

  if (s_sweepOn) {
    lv_draw_line_dsc_t beam;
    lv_draw_line_dsc_init(&beam);
    beam.color = kSweep;
    beam.width = 2;
    beam.p1 = {static_cast<lv_value_precise_t>(cx), static_cast<lv_value_precise_t>(cy)};
    for (size_t i = 0; i < sizeof kSweepOpa; ++i) {
      const float ang = s_sweepAngle - i * kSweepGap;
      const float x = kScopeR * std::sin(ang), y = -kScopeR * std::cos(ang);
      const lv_area_t bounds = {static_cast<int32_t>(std::min(0.0f, x)) - 2, static_cast<int32_t>(std::min(0.0f, y)) - 2,
                                static_cast<int32_t>(std::max(0.0f, x)) + 2, static_cast<int32_t>(std::max(0.0f, y)) + 2};
      if (!inStrip(bounds)) continue;
      beam.opa = kSweepOpa[i];
      beam.p2 = pt(cx + x, cy + y);
      lv_draw_line(layer, &beam);
    }
  }

  lv_draw_line_dsc_t trail;
  lv_draw_line_dsc_init(&trail);
  trail.width = 1;
  trail.opa = LV_OPA_50;
  for (const Segment& seg : s_layout.trails) {
    const lv_area_t bounds = {static_cast<int32_t>(std::min(seg.p0.x, seg.p1.x)),
                              static_cast<int32_t>(std::min(seg.p0.y, seg.p1.y)),
                              static_cast<int32_t>(std::max(seg.p0.x, seg.p1.x)),
                              static_cast<int32_t>(std::max(seg.p0.y, seg.p1.y))};
    if (!inStrip(bounds)) continue;
    trail.color = seg.color;
    trail.p1 = {seg.p0.x + cx, seg.p0.y + cy};
    trail.p2 = {seg.p1.x + cx, seg.p1.y + cy};
    lv_draw_line(layer, &trail);
  }

  for (const Badge& b : s_layout.badges) {
    if (inStrip(b.area)) drawBadge(layer, offset(b.area, cx, cy), b.count);
  }

  lv_draw_triangle_dsc_t tri;
  lv_draw_triangle_dsc_init(&tri);
  tri.bg_opa = LV_OPA_COVER;
  for (const Marker& m : s_layout.markers) {
    constexpr int kReach = 15;  // the selection ring is the widest part
    const lv_area_t bounds = {m.x - kReach, m.y - kReach, m.x + kReach, m.y + kReach};
    if (!inStrip(bounds)) continue;
    const float px = cx + m.x, py = cy + m.y;
    // Icon coordinates: x to the right of the nose, y forward along the track.
    const float heading = m.arrow ? m.track : 0;
    const float c = std::cos(heading), sn = std::sin(heading);
    auto at = [&](float x, float y) { return pt(px + x * c + y * sn, py + x * sn - y * c); };
    if (m.heli) {
      drawRing(layer, px, py, 2, 3, m.color);
      lv_draw_line_dsc_t l;
      lv_draw_line_dsc_init(&l);
      l.color = m.color;
      l.width = 2;
      l.p1 = at(0, 0);
      l.p2 = at(0, -8);  // tail boom
      lv_draw_line(layer, &l);
      l.width = 1;
      l.p1 = at(-5, 5);  // rotor
      l.p2 = at(5, -5);
      lv_draw_line(layer, &l);
      l.p1 = at(5, 5);
      l.p2 = at(-5, -5);
      lv_draw_line(layer, &l);
    } else if (m.arrow) {
      tri.bg_color = m.color;
      constexpr float kPlane[][3][2] = {
          {{0, 9}, {-1.5f, -6}, {1.5f, -6}},   // fuselage
          {{0, 3}, {-8, -2}, {8, -2}},         // wings
          {{0, -3}, {-3.5f, -7}, {3.5f, -7}},  // tail
      };
      for (const auto& t : kPlane) {
        for (int k = 0; k < 3; ++k) tri.p[k] = at(t[k][0], t[k][1]);
        lv_draw_triangle(layer, &tri);
      }
    } else {
      drawRing(layer, px, py, m.ground ? 2 : 4, m.ground ? 3 : 4, m.color);
    }
    if (m.selected) drawRing(layer, px, py, 13, 2, kSelect);
  }

  for (const Callout& c : s_layout.callouts) {
    if (!inStrip(c.area)) continue;
    drawLabel(layer, c.text.c_str(), kRadarFont, c.color, c.area.x1 + cx, c.area.y1 + cy,
              lv_area_get_width(&c.area));
  }
}

// Marks the wedge between two angles as changed, in a few pieces along the
// radius: one box around a diagonal line would cover a whole quarter of the scope.
void invalidateWedge(float from, float to) {
  constexpr int kPieces = 5;
  constexpr float kStep = 5 * 3.14159265f / 180;
  lv_area_t box;
  lv_obj_get_coords(s_scope, &box);
  const int cx = box.x1 + kScope / 2, cy = box.y1 + kScope / 2;
  for (int k = 0; k < kPieces; ++k) {
    float x1 = 1e9f, y1 = 1e9f, x2 = -1e9f, y2 = -1e9f;
    for (int r : {kScopeR * k / kPieces, kScopeR * (k + 1) / kPieces}) {
      for (float ang = from;; ang = std::min(ang + kStep, to)) {
        const float x = r * std::sin(ang), y = -r * std::cos(ang);
        x1 = std::min(x1, x), y1 = std::min(y1, y), x2 = std::max(x2, x), y2 = std::max(y2, y);
        if (ang >= to) break;
      }
    }
    const lv_area_t a = {cx + static_cast<int32_t>(std::floor(x1)) - 2, cy + static_cast<int32_t>(std::floor(y1)) - 2,
                         cx + static_cast<int32_t>(std::ceil(x2)) + 2, cy + static_cast<int32_t>(std::ceil(y2)) + 2};
    lv_obj_invalidate_area(s_scope, &a);
  }
}

void sweepTick(lv_timer_t*) {
  constexpr float kTwoPi = 2 * 3.14159265f;
  if (!s_sweepOn) return;
  const float was = s_sweepAngle;
  s_sweepAngle = kTwoPi * (esp_timer_get_time() % kSweepPeriodUs) / kSweepPeriodUs;
  float moved = s_sweepAngle - was;
  if (moved < 0) moved += kTwoPi;
  // After a long stall the wedge is most of the circle; just repaint it all.
  const bool stalled = moved > kTwoPi / 8;
  if (stalled) {
    lv_obj_invalidate(s_scope);
  } else {
    invalidateWedge(was - kSweepTrail, was + moved);
  }

  const int64_t now = esp_timer_get_time();
  bool repaint = false;
  for (const Aircraft& a : s.snap->aircraft) {
    Point p;
    shownPosition(a, now, p.eastNm, p.northNm);
    float behind = std::fmod(s_sweepAngle - std::atan2(p.eastNm, p.northNm) + 2 * kTwoPi, kTwoPi);
    if (!stalled && behind >= moved) continue;
    s.painted[a.hex] = p;
    repaint = true;
  }
  if (repaint) updateScope();
}

void applySweep(bool on) {
  if (on == s_sweepOn) return;
  if (s_sweepOn) invalidateWedge(s_sweepAngle - kSweepTrail, s_sweepAngle);
  s_sweepOn = on;
  // Switching it on shouldn't blank the scope until the beam comes round.
  s.painted.clear();
  const int64_t now = esp_timer_get_time();
  for (const Aircraft& a : s.snap->aircraft) {
    Point& p = s.painted[a.hex];
    shownPosition(a, now, p.eastNm, p.northNm);
  }
  lv_obj_set_state(s_sweepBtn, LV_STATE_CHECKED, on);
}

void onSweepClicked(lv_event_t*) {
  const bool on = !s_sweepOn;
  setSweep(on);
  if (s_sweepChosen) s_sweepChosen(on);
}

const Aircraft* findSelected() {
  for (const Aircraft& a : s.snap->aircraft) {
    if (a.hex == s.selected) return &a;
  }
  return nullptr;
}

void setHidden(lv_obj_t* o, bool hidden) {
  if (hidden) {
    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
  }
}

// The top of the panel shows settings, else the selected aircraft, else the list.
void showView() {
  const bool detail = !s.configOpen && !s.selected.empty();
  setHidden(s_config, !s.configOpen);
  setHidden(s_detail, !detail);
  setHidden(s_list, s.configOpen || detail);
  lv_obj_set_state(s_configBtn, LV_STATE_CHECKED, s.configOpen);
}

void refreshDetail() {
  const Aircraft* a = findSelected();
  if (!a) s.selected.clear();
  showView();
  if (!a) return;
  lv_label_set_text(s_detailTitle, a->label().c_str());
  lv_obj_set_style_text_color(s_detailTitle, altColor(*a), 0);

  std::string body;
  char line[64];
  if (!a->reg.empty() || !a->type.empty()) {
    snprintf(line, sizeof line, "%s  %s\n", a->reg.c_str(), a->type.c_str());
    body += line;
  }
  if (a->onGround) {
    body += "On ground\n";
  } else if (a->hasAlt) {
    if (s.metric) {
      snprintf(line, sizeof line, "%ld m%s\n", std::lround(a->altFt * 0.3048f), climbSymbol(*a));
    } else {
      snprintf(line, sizeof line, "%d ft%s\n", a->altFt, climbSymbol(*a));
    }
    body += line;
    if (std::abs(a->vrateFpm) > 300) {
      if (s.metric) {
        snprintf(line, sizeof line, "%+.1f m/s\n", a->vrateFpm * 0.00508f);
      } else {
        snprintf(line, sizeof line, "%+d fpm\n", a->vrateFpm);
      }
      body += line;
    }
  }
  if (a->gsKt >= 0) {
    snprintf(line, sizeof line, s.metric ? "%.0f km/h" : "%.0f kt", s.metric ? a->gsKt * 1.852f : a->gsKt);
    body += line;
    if (a->hasTrack) {
      snprintf(line, sizeof line, "  trk %03.0f", a->track);
      body += line;
    }
    body += "\n";
  }
  snprintf(line, sizeof line, "%.1f%s  brg %03.0f\n", distOut(a->distNm), distUnit(), a->bearing);
  body += line;
  if (!a->squawk.empty()) body += "Squawk " + a->squawk + "\n";
  if (a->emergency) body += "EMERGENCY\n";
  body += "Hex " + a->hex;
  lv_label_set_text(s_detailBody, body.c_str());
}

void onRowClicked(lv_event_t* e) {
  const size_t i = reinterpret_cast<uintptr_t>(lv_event_get_user_data(e));
  if (i >= s.rowHex.size()) return;
  s.selected = s.rowHex[i];
  refreshDetail();
  updateScope();
}

void onDetailClicked(lv_event_t*) {
  s.selected.clear();
  refreshDetail();
  updateScope();
}

void rebuildList() {
  s.rowHex.clear();
  // What the radar shows: airborne first (nearest first, as the snapshot is
  // sorted), then parked.
  const float limit = radarLimitNm();
  std::vector<const Aircraft*> rows;
  for (const Aircraft& a : s.snap->aircraft) {
    if (!a.onGround && a.distNm <= limit) rows.push_back(&a);
  }
  if (s.ground != Ground::Hide) {
    for (const Aircraft& a : s.snap->aircraft) {
      if (a.onGround && a.distNm <= limit) rows.push_back(&a);
    }
  }
  for (int i = 0; i < kMaxRows; ++i) {
    Row& r = s_rows[i];
    if (i >= static_cast<int>(rows.size())) {
      if (!lv_obj_has_flag(r.obj, LV_OBJ_FLAG_HIDDEN)) lv_obj_add_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    const Aircraft& a = *rows[i];
    std::string alt = altText(a) + climbSymbol(a);
    char dist[12];
    rangeText(dist, sizeof dist, a.distNm, "");
    if (r.name != a.label() || !lv_color_eq(r.color, altColor(a)) || r.alt != alt || r.dist != dist) {
      r.name = a.label();
      r.color = altColor(a);
      r.alt = std::move(alt);
      r.dist = dist;
      lv_obj_invalidate(r.obj);
    }
    if (lv_obj_has_flag(r.obj, LV_OBJ_FLAG_HIDDEN)) lv_obj_remove_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
    s.rowHex.push_back(a.hex);
  }
}

// One field of a row, vertically centred. The name is cut to its column rather
// than wrapped, as a second line would show under the row.
void rowText(lv_layer_t* layer, const char* text, const lv_font_t* font, lv_color_t color, int x, int y, int w,
             lv_text_align_t align) {
  lv_draw_label_dsc_t d;
  lv_draw_label_dsc_init(&d);
  d.text = text;
  d.font = font;
  d.color = color;
  d.align = align;
  d.flag = LV_TEXT_FLAG_EXPAND;
  std::string cut;
  lv_point_t size;
  lv_text_get_size(&size, text, font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  if (size.x > w) {
    cut = text;
    while (!cut.empty() && size.x > w) {
      cut.pop_back();
      lv_text_get_size(&size, cut.c_str(), font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    }
    d.text = cut.c_str();
  }
  // With EXPAND the text is laid out at its own width, so align by hand.
  const int tx = align == LV_TEXT_ALIGN_RIGHT ? x + w - size.x : x;
  const int ty = y + (kRowH - lv_font_get_line_height(font)) / 2;
  lv_area_t area = {tx, ty, tx + size.x, ty + lv_font_get_line_height(font)};
  lv_draw_label(layer, &d, &area);
}

// A row draws its own three fields: each LVGL widget costs a fixed few hundred
// microseconds per repaint, so three labels per row made the list the slowest
// thing on the screen.
void drawRow(lv_event_t* e) {
  const Row& r = s_rows[reinterpret_cast<uintptr_t>(lv_event_get_user_data(e))];
  lv_layer_t* layer = lv_event_get_layer(e);
  lv_area_t box;
  lv_obj_get_coords(r.obj, &box);
  const int x = box.x1 + 3, w = kInnerW - 6;
  rowText(layer, r.name.c_str(), &lv_font_montserrat_14, r.color, x, box.y1, 74, LV_TEXT_ALIGN_LEFT);
  rowText(layer, r.alt.c_str(), &lv_font_montserrat_12, kText, x + 74, box.y1, 52, LV_TEXT_ALIGN_RIGHT);
  rowText(layer, r.dist.c_str(), &lv_font_montserrat_12, kDim, x + 126, box.y1, w - 126, LV_TEXT_ALIGN_RIGHT);
}

// One line per aircraft: callsign, altitude, distance in nm. Type and the rest
// are in the details.
void createRows() {
  for (int i = 0; i < kMaxRows; ++i) {
    lv_obj_t* row = lv_obj_create(s_list);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), kRowH);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(row, kDivider, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_bg_color(row, kButtonPressed, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_HIDDEN));
    void* index = reinterpret_cast<void*>(static_cast<uintptr_t>(i));
    lv_obj_add_event_cb(row, onRowClicked, LV_EVENT_CLICKED, index);
    lv_obj_add_event_cb(row, drawRow, LV_EVENT_DRAW_MAIN, index);
    s_rows[i].obj = row;
  }
}

void updateRange() {
  char buf[16];
  rangeText(buf, sizeof buf, viewNm(), distUnit());
  lv_label_set_text(s_range, buf);
  lv_obj_set_state(s_zoomOut, LV_STATE_DISABLED, s.zoom == 0);
  lv_obj_set_state(s_zoomIn, LV_STATE_DISABLED, s.zoom == kZoomLevels - 1);
}

void applyZoom(int zoom) {
  s.zoom = std::clamp(zoom, 0, kZoomLevels - 1);
  updateRange();
  rebuildList();
  updateScope();
}

void applyUnits(bool metric) {
  if (metric == s.metric) return;
  s.metric = metric;
  lv_obj_set_state(s_metricBtn, LV_STATE_CHECKED, metric);
  updateRange();
  rebuildList();
  if (!s.selected.empty()) refreshDetail();
  s_bgNm = -1;  // the ring labels
  renderBackground();
  lv_obj_invalidate(s_scope);
}

void onMetricClicked(lv_event_t*) {
  const bool metric = !s.metric;
  setUnits(metric);
  if (s_unitsChosen) s_unitsChosen(metric);
}

// A zoom the user picked, as opposed to one restored from settings.
void chooseZoom(int zoom) {
  applyZoom(zoom);
  if (s_zoomChosen) s_zoomChosen(s.zoom);
}

void onZoomClicked(lv_event_t* e) {
  chooseZoom(s.zoom + static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e))));
}

void updateGroundButtons() {
  for (int i = 0; i < 3; ++i) lv_obj_set_state(s_groundOpts[i], LV_STATE_CHECKED, s.ground == kGroundModes[i]);
}

void onGroundClicked(lv_event_t* e) {
  const Ground mode = kGroundModes[reinterpret_cast<intptr_t>(lv_event_get_user_data(e))];
  setGround(mode);
  if (s_groundChosen) s_groundChosen(mode);
}

void onConfigClicked(lv_event_t*) {
  s.configOpen = !s.configOpen;
  showView();
}

void updateTrails() {
  std::unordered_map<std::string, std::deque<Point>> next;
  for (const Aircraft& a : s.snap->aircraft) {
    auto& pts = next[a.hex];
    auto it = s.trails.find(a.hex);
    if (it != s.trails.end()) pts = std::move(it->second);
    Point p;
    position(a, p.eastNm, p.northNm);
    if (pts.empty() || std::hypot(p.eastNm - pts.back().eastNm, p.northNm - pts.back().northNm) > 0.02f) {
      pts.push_back(p);
      if (pts.size() > kTrailLen) pts.pop_front();
    }
  }
  s.trails = std::move(next);
}

void onScopeClicked(lv_event_t*) {
  lv_point_t pt;
  lv_indev_get_point(lv_indev_active(), &pt);
  lv_area_t box;
  lv_obj_get_coords(s_scope, &box);
  const float tx = pt.x - (box.x1 + kScope / 2), ty = pt.y - (box.y1 + kScope / 2);
  const Aircraft* best = nullptr;
  float bestD = kPickRadiusPx;
  const int64_t now = esp_timer_get_time();
  for (const Aircraft& a : s.snap->aircraft) {
    // A tap on a ground badge falls through to zoom, which splits the crowd.
    if (!drawnSingly(a)) continue;
    float east, north, x, y;
    if (!drawnPosition(a, now, east, north)) continue;
    toScope(east, north, x, y);
    const float d = std::hypot(x - tx, y - ty);
    if (d < bestD) {
      bestD = d;
      best = &a;
    }
  }
  if (best) {
    s.selected = best->hex;
    s.configOpen = false;
  } else {
    chooseZoom((s.zoom + 1) % kZoomLevels);
  }
  refreshDetail();
  updateScope();
}

int flyingNow() {
  return std::count_if(s.snap->aircraft.begin(), s.snap->aircraft.end(),
                       [](const Aircraft& a) { return !a.onGround; });
}

// Different aircraft seen flying in the last hour. In PSRAM, as internal RAM is tight.
struct Seen {
  uint32_t icao;
  uint32_t minute;
};
constexpr int kSeenCap = 2048;
constexpr uint32_t kHourMin = 60;
Seen* s_seen;
int s_seenCount;

// The in-the-air count once a minute, for the hourly average.
int s_samples[kHourMin];
int s_sampleCount;
int s_sampleNext;
uint32_t s_sampleMinute = UINT32_MAX;

uint32_t icaoKey(const std::string& hex) {
  const bool tisb = !hex.empty() && hex[0] == '~';  // non-ICAO address
  return strtoul(hex.c_str() + tisb, nullptr, 16) | (tisb ? 0x1000000u : 0);
}

void recordSeen() {
  if (!s_seen) return;
  const uint32_t now = esp_timer_get_time() / 60000000;
  for (const Aircraft& a : s.snap->aircraft) {
    if (a.onGround) continue;
    const uint32_t key = icaoKey(a.hex);
    Seen* end = s_seen + s_seenCount;
    Seen* it = std::find_if(s_seen, end, [key](const Seen& e) { return e.icao == key; });
    if (it != end) {
      it->minute = now;
    } else if (s_seenCount < kSeenCap) {
      s_seen[s_seenCount++] = {key, now};
    }
  }
  Seen* end = std::remove_if(s_seen, s_seen + s_seenCount, [now](const Seen& e) { return now - e.minute >= kHourMin; });
  s_seenCount = end - s_seen;
  if (now != s_sampleMinute) {
    s_sampleMinute = now;
    s_samples[s_sampleNext] = flyingNow();
    s_sampleNext = (s_sampleNext + 1) % kHourMin;
    s_sampleCount = std::min<int>(s_sampleCount + 1, kHourMin);
  }
}

void updateStatus() {
  const int64_t now = esp_timer_get_time();
  char buf[96];
  if (s.net == Net::Unprovisioned) {
    snprintf(buf, sizeof buf, "No Wi-Fi\nset up");
  } else if (s.net == Net::Connecting) {
    snprintf(buf, sizeof buf, "Wi-Fi\nconnecting");
  } else if (!s.error.empty() && s.snap->fetchedUs == 0) {
    snprintf(buf, sizeof buf, "%s", s.error.c_str());
  } else if (s.snap->fetchedUs == 0) {
    snprintf(buf, sizeof buf, "Fetching");
  } else {
    const int age = static_cast<int>((now - s.snap->fetchedUs) / 1000000);
    // Zoomed in, the radar shows fewer than the list, so say how many of them it shows.
    char count[40];
    if (s.zoom == 0) {
      snprintf(count, sizeof count, "%d in the air", flyingNow());
    } else {
      const float limit = radarLimitNm();
      const auto inView = std::count_if(s.snap->aircraft.begin(), s.snap->aircraft.end(),
                                        [limit](const Aircraft& a) { return !a.onGround && a.distNm <= limit; });
      snprintf(count, sizeof count, "%d of %d in air", static_cast<int>(inView), flyingNow());
    }
    snprintf(buf, sizeof buf, "%s\n%s%ds ago", count, s.error.empty() ? "" : "STALE ", age);
  }
  setText(s_status, buf);
  setTextColor(s_status, s.error.empty() || s.snap->fetchedUs == 0 ? kDim : kEmergency);
}

// Provider, request time, HTTP status if not 200 and any rate-limit headers the
// provider sends; then the hour's average in the air and how many different
// aircraft flew (both since boot, if that's sooner).
void updateLink() {
  const feed::Status st = feed::status();
  std::string text = feed::provider();
  char part[32];
  if (st.fetchMs >= 0) {
    snprintf(part, sizeof part, "  %.1fs", st.fetchMs / 1000.0f);
    text += part;
  }
  if (st.httpStatus != 0 && st.httpStatus != 200) {
    snprintf(part, sizeof part, "  HTTP %d", st.httpStatus);
    text += part;
  }
  if (st.rateRemaining >= 0) {
    snprintf(part, sizeof part, "  %d left", st.rateRemaining);
    text += part;
  }
  if (st.retryAfterS > 0) {
    snprintf(part, sizeof part, "  wait %ds", st.retryAfterS);
    text += part;
  }
  if (s_sampleCount > 0) {
    const int sum = std::accumulate(s_samples, s_samples + s_sampleCount, 0);
    snprintf(part, sizeof part, "\nAvg %d  Flights 1h %d", (sum + s_sampleCount / 2) / s_sampleCount, s_seenCount);
  } else {
    snprintf(part, sizeof part, "\nFlights 1h %d", s_seenCount);
  }
  text += part;
  if (text != lv_label_get_text(s_link)) lv_label_set_text(s_link, text.c_str());
}

void updateClock() {
  time_t t = time(nullptr);
  struct tm tm;
  localtime_r(&t, &tm);
  // RTC keeps time across soft resets, so a plausible year means synced.
  const int minute = tm.tm_year + 1900 >= 2025 ? tm.tm_hour * 60 + tm.tm_min : -2;
  if (minute == s.lastMinute) return;
  s.lastMinute = minute;
  if (minute < 0) {
    lv_label_set_text(s_clock, "--:--");
  } else {
    char buf[8];
    strftime(buf, sizeof buf, "%H:%M", &tm);
    lv_label_set_text(s_clock, buf);
  }
}

void tick(lv_timer_t*) {
  bool homeDirty, groundChanged, zoomChanged, sweep, metric;
  {
    std::lock_guard<std::mutex> g(s_inMutex);
    s.net = s_inNet;
    if (s.message != s_inMessage) {
      s.message = s_inMessage;
      lv_label_set_text(s_scopeMsg, s.message.c_str());
      if (s.message.empty()) {
        lv_obj_add_flag(s_scopeMsg, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_remove_flag(s_scopeMsg, LV_OBJ_FLAG_HIDDEN);
      }
    }
    homeDirty = s_inHomeDirty;
    s_inHomeDirty = false;
    if (homeDirty) {
      s.homeLat = s_inLat;
      s.homeLon = s_inLon;
      s.radiusNm = s_inRadius;
    }
    groundChanged = s.ground != s_inGround;
    s.ground = s_inGround;
    zoomChanged = s.zoom != s_inZoom;
    if (zoomChanged) s.zoom = s_inZoom;
    sweep = s_inSweep;
    metric = s_inMetric;
  }
  applySweep(sweep);
  applyUnits(metric);
  if (zoomChanged) applyZoom(s.zoom);
  if (homeDirty) {
    s.trails.clear();
    updateRange();
  }
  if (groundChanged) {
    const Aircraft* sel = findSelected();
    if (sel && sel->onGround && s.ground == Ground::Hide) s.selected.clear();
    updateGroundButtons();
    rebuildList();
    refreshDetail();
  }
  s.error = feed::lastError();
  const uint32_t gen = feed::generation();
  const int64_t now = esp_timer_get_time();
  if (gen != s.generation) {
    s.generation = gen;
    std::unordered_map<std::string, Point> shown;
    for (const Aircraft& a : s.snap->aircraft) {
      if (a.onGround) continue;
      Point& p = shown[a.hex];
      shownPosition(a, now, p.eastNm, p.northNm);
    }
    s.snap = feed::latest();
    startEasing(shown, now);
    std::unordered_map<std::string, Point> painted;
    for (const Aircraft& a : s.snap->aircraft) {
      auto it = s.painted.find(a.hex);
      if (it != s.painted.end()) painted.emplace(a.hex, it->second);
    }
    s.painted = std::move(painted);
    recordSeen();
    updateTrails();
    rebuildList();
    if (!s.selected.empty()) refreshDetail();
  }
  updateScope();  // every tick, as aircraft are carried on between polls
  updateStatus();
  updateLink();
  updateClock();
}

lv_obj_t* plainBox(lv_obj_t* parent) {
  lv_obj_t* o = lv_obj_create(parent);
  lv_obj_set_style_radius(o, 0, 0);
  lv_obj_set_style_border_width(o, 0, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  return o;
}

constexpr int kButtonH = 28;

lv_obj_t* button(lv_obj_t* parent, const char* text, const lv_font_t* font, int x, int w, lv_event_cb_t cb,
                 intptr_t arg) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_size(b, w, kButtonH);
  lv_obj_set_style_radius(b, 6, 0);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(b, kButton, 0);
  lv_obj_set_style_bg_color(b, kButtonPressed, LV_STATE_PRESSED);
  lv_obj_set_style_opa(b, LV_OPA_40, LV_STATE_DISABLED);
  lv_obj_align(b, LV_ALIGN_BOTTOM_LEFT, x, -2);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, reinterpret_cast<void*>(arg));
  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, text);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, kText, 0);
  lv_obj_center(l);
  return b;
}

}  // namespace

void init(double homeLat, double homeLon, int radiusNm) {
  display::Guard lock;
  s.homeLat = homeLat;
  s.homeLon = homeLon;
  s.radiusNm = radiusNm;
  s_seen = static_cast<Seen*>(heap_caps_malloc(kSeenCap * sizeof(Seen), MALLOC_CAP_SPIRAM));

  lv_obj_t* screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, kBg, 0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  s_scope = plainBox(screen);
  lv_obj_set_pos(s_scope, 0, 0);
  lv_obj_set_size(s_scope, kScope, kScope);
  lv_obj_set_style_bg_color(s_scope, kBg, 0);
  lv_obj_remove_flag(s_scope, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(s_scope, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(s_scope, drawScope, LV_EVENT_DRAW_MAIN, nullptr);
  lv_obj_add_event_cb(s_scope, onScopeClicked, LV_EVENT_CLICKED, nullptr);

  void* bgPixels = heap_caps_malloc(kScope * kScope * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
  assert(bgPixels);
  lv_draw_buf_init(&s_bgBuf, kScope, kScope, LV_COLOR_FORMAT_RGB565, kScope * sizeof(uint16_t), bgPixels,
                   kScope * kScope * sizeof(uint16_t));
  s_bgCanvas = lv_canvas_create(screen);
  lv_canvas_set_draw_buf(s_bgCanvas, &s_bgBuf);
  lv_obj_add_flag(s_bgCanvas, LV_OBJ_FLAG_HIDDEN);  // only ever drawn into
  renderBackground();

  s_scopeMsg = lv_label_create(s_scope);
  lv_obj_set_width(s_scopeMsg, kScope - 80);  // stays inside the round window
  lv_label_set_long_mode(s_scopeMsg, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(s_scopeMsg, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(s_scopeMsg, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(s_scopeMsg, kText, 0);
  lv_obj_set_style_bg_color(s_scopeMsg, kPanelBg, 0);
  lv_obj_set_style_bg_opa(s_scopeMsg, LV_OPA_80, 0);
  lv_obj_set_style_pad_all(s_scopeMsg, 8, 0);
  lv_obj_center(s_scopeMsg);
  lv_obj_add_flag(s_scopeMsg, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* panel = plainBox(screen);
  lv_obj_set_pos(panel, kPanelX, 0);
  lv_obj_set_size(panel, kPanelW, board::kHeight);
  lv_obj_set_style_bg_color(panel, kPanelBg, 0);
  lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

  s_list = plainBox(panel);
  lv_obj_set_pos(s_list, kInset, kInset);
  lv_obj_set_size(s_list, kInnerW, kListH);
  lv_obj_set_style_bg_color(s_list, kPanelBg, 0);
  lv_obj_set_flex_flow(s_list, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(s_list, 0, 0);
  lv_obj_set_scroll_dir(s_list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(s_list, LV_SCROLLBAR_MODE_ACTIVE);
  createRows();

  s_detail = plainBox(panel);
  lv_obj_set_pos(s_detail, kInset, kInset);
  lv_obj_set_size(s_detail, kInnerW, kListH);
  lv_obj_set_style_bg_color(s_detail, kPanelBg, 0);
  lv_obj_set_style_pad_all(s_detail, 4, 0);
  lv_obj_remove_flag(s_detail, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(s_detail, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_HIDDEN));
  lv_obj_add_event_cb(s_detail, onDetailClicked, LV_EVENT_CLICKED, nullptr);

  s_detailTitle = lv_label_create(s_detail);
  lv_obj_set_style_text_font(s_detailTitle, &lv_font_montserrat_16, 0);
  lv_obj_align(s_detailTitle, LV_ALIGN_TOP_LEFT, 0, 0);

  s_detailBody = lv_label_create(s_detail);
  lv_obj_set_width(s_detailBody, kInnerW - 8);
  lv_obj_set_style_text_font(s_detailBody, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(s_detailBody, kText, 0);
  lv_obj_set_style_text_line_space(s_detailBody, 2, 0);
  lv_obj_align(s_detailBody, LV_ALIGN_TOP_LEFT, 0, 24);

  lv_obj_t* hint = lv_label_create(s_detail);
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(hint, kDim, 0);
  lv_label_set_text(hint, "tap to go back");
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, 0);

  s_config = plainBox(panel);
  lv_obj_set_pos(s_config, kInset, kInset);
  lv_obj_set_size(s_config, kInnerW, kListH);
  lv_obj_set_style_bg_color(s_config, kPanelBg, 0);
  lv_obj_set_style_pad_all(s_config, 4, 0);
  lv_obj_remove_flag(s_config, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(s_config, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* title = lv_label_create(s_config);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(title, kText, 0);
  lv_label_set_text(title, "Settings");
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t* groundTitle = lv_label_create(s_config);
  lv_obj_set_style_text_font(groundTitle, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(groundTitle, kDim, 0);
  lv_label_set_text(groundTitle, "Aircraft on the ground");
  lv_obj_align(groundTitle, LV_ALIGN_TOP_LEFT, 0, 28);
  for (int i = 0; i < 3; ++i) {
    s_groundOpts[i] = button(s_config, kGroundNames[i], &lv_font_montserrat_14, 0, kInnerW - 8, onGroundClicked, i);
    lv_obj_align(s_groundOpts[i], LV_ALIGN_TOP_LEFT, 0, 46 + i * (kButtonH + 4));
    lv_obj_set_style_bg_color(s_groundOpts[i], kRing, LV_STATE_CHECKED);
  }
  constexpr int kHalfW = (kInnerW - 8 - 4) / 2, kTogglesY = 46 + 3 * (kButtonH + 4) + 6;
  s_sweepBtn = button(s_config, "Sweep", &lv_font_montserrat_14, 0, kHalfW, onSweepClicked, 0);
  lv_obj_align(s_sweepBtn, LV_ALIGN_TOP_LEFT, 0, kTogglesY);
  lv_obj_set_style_bg_color(s_sweepBtn, kRing, LV_STATE_CHECKED);
  s_metricBtn = button(s_config, "Metric", &lv_font_montserrat_14, 0, kHalfW, onMetricClicked, 0);
  lv_obj_align(s_metricBtn, LV_ALIGN_TOP_LEFT, kHalfW + 4, kTogglesY);
  lv_obj_set_style_bg_color(s_metricBtn, kRing, LV_STATE_CHECKED);
  lv_obj_t* done = button(s_config, "Done", &lv_font_montserrat_14, 0, kInnerW - 8, onConfigClicked, 0);
  lv_obj_align(done, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  lv_obj_t* controls = plainBox(panel);
  lv_obj_set_pos(controls, kInset, kInset + kListH);
  lv_obj_set_size(controls, kInnerW, kControlsH);
  lv_obj_set_style_bg_opa(controls, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_side(controls, LV_BORDER_SIDE_TOP, 0);
  lv_obj_set_style_border_color(controls, kDivider, 0);
  lv_obj_set_style_border_width(controls, 1, 0);
  lv_obj_remove_flag(controls, LV_OBJ_FLAG_SCROLLABLE);

  s_clock = lv_label_create(controls);
  lv_obj_set_style_text_font(s_clock, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(s_clock, kText, 0);
  lv_label_set_text(s_clock, "--:--");
  lv_obj_align(s_clock, LV_ALIGN_TOP_LEFT, 2, 6);

  s_status = lv_label_create(controls);
  lv_obj_set_size(s_status, 84, 2 * lv_font_get_line_height(&lv_font_montserrat_12));
  lv_label_set_long_mode(s_status, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_font(s_status, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(s_status, kDim, 0);
  lv_obj_set_style_text_align(s_status, LV_TEXT_ALIGN_RIGHT, 0);
  lv_label_set_text(s_status, "");
  lv_obj_align(s_status, LV_ALIGN_TOP_RIGHT, -2, 2);

  s_link = lv_label_create(controls);
  lv_obj_set_size(s_link, kInnerW - 4, 2 * lv_font_get_line_height(&lv_font_montserrat_12));
  lv_label_set_long_mode(s_link, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_font(s_link, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(s_link, kDim, 0);
  lv_label_set_text(s_link, "");
  lv_obj_align(s_link, LV_ALIGN_TOP_LEFT, 2, 34);

  // [-] range [+] [settings]; "+" zooms in (a smaller range), as on a map.
  constexpr int kZoomW = 32, kRangeW = 56, kConfigW = kInnerW - 2 * kZoomW - kRangeW - 2;
  s_zoomOut = button(controls, LV_SYMBOL_MINUS, &lv_font_montserrat_14, 0, kZoomW, onZoomClicked, -1);
  s_zoomIn = button(controls, LV_SYMBOL_PLUS, &lv_font_montserrat_14, kZoomW + kRangeW, kZoomW, onZoomClicked, 1);
  s_configBtn =
      button(controls, LV_SYMBOL_SETTINGS, &lv_font_montserrat_14, kInnerW - kConfigW, kConfigW, onConfigClicked, 0);
  lv_obj_set_style_bg_color(s_configBtn, kButtonPressed, LV_STATE_CHECKED);
  s_range = lv_label_create(controls);
  lv_obj_set_width(s_range, kRangeW);
  lv_obj_set_style_text_align(s_range, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(s_range, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(s_range, kRingText, 0);
  lv_obj_align(s_range, LV_ALIGN_BOTTOM_LEFT, kZoomW, -8);
  updateRange();
  updateGroundButtons();

  lv_timer_create(tick, 250, nullptr);
  lv_timer_create(sweepTick, kSweepFrameMs, nullptr);
}

void setHome(double lat, double lon, int radiusNm) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inLat = lat;
  s_inLon = lon;
  s_inRadius = radiusNm;
  s_inHomeDirty = true;
}

void setGround(Ground mode) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inGround = mode;
}

void onGroundChosen(void (*cb)(Ground mode)) { s_groundChosen = cb; }

void setZoom(int level) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inZoom = std::clamp(level, 0, kZoomLevels - 1);
}

void onZoomChosen(void (*cb)(int level)) { s_zoomChosen = cb; }

void setSweep(bool on) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inSweep = on;
}

void onSweepChosen(void (*cb)(bool on)) { s_sweepChosen = cb; }

void setUnits(bool metric) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inMetric = metric;
}

void onUnitsChosen(void (*cb)(bool metric)) { s_unitsChosen = cb; }

void setNet(Net net, const std::string& message) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inNet = net;
  s_inMessage = message;
}

}  // namespace ui
