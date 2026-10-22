#include "Ui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
  int zoom = 0;
  std::shared_ptr<const Snapshot> snap = std::make_shared<Snapshot>();
  uint32_t generation = 0;
  std::string selected;  // hex
  std::unordered_map<std::string, std::deque<Point>> trails;
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

// List rows are created once and refilled, so a poll arriving mid-scroll
// doesn't delete the row under the finger.
struct Row {
  lv_obj_t* obj;
  lv_obj_t* name;
  lv_obj_t* alt;
  lv_obj_t* dist;
};

lv_obj_t* s_scope;
lv_obj_t* s_scopeMsg;
lv_obj_t* s_clock;
lv_obj_t* s_status;
lv_obj_t* s_link;
lv_obj_t* s_range;
lv_obj_t* s_zoomOut;
lv_obj_t* s_zoomIn;
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

void rangeText(char* buf, size_t len, float nm, const char* suffix) {
  snprintf(buf, len, nm < 10 ? "%.1f%s" : "%.0f%s", nm, suffix);
}

const Aircraft* findSelected();

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

std::string altText(const Aircraft& a) {
  if (!a.hasAlt) return "";
  if (a.onGround) return "GND";
  char buf[12];
  if (a.altFt >= 18000) {
    snprintf(buf, sizeof buf, "FL%03d", (a.altFt + 50) / 100);
  } else {
    snprintf(buf, sizeof buf, "%d", (a.altFt + 50) / 100 * 100);
  }
  return buf;
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
// Returns the area it covers, so labels keep clear of it.
lv_area_t drawCluster(lv_layer_t* layer, float x, float y, int count) {
  const int ix = std::lround(x), iy = std::lround(y);
  if (count == 1) {
    drawRing(layer, x, y, 2, 3, lv_color_hex(0x9a9a9a));
    return {ix - 3, iy - 3, ix + 3, iy + 3};
  }
  char text[8];
  snprintf(text, sizeof text, "%d", count);
  lv_point_t size;
  lv_text_get_size(&size, text, &lv_font_montserrat_12, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
  const int w = std::max<int>(size.x + 8, size.y + 2), h = size.y + 2;
  const lv_area_t area = {ix - w / 2, iy - h / 2, ix - w / 2 + w - 1, iy - h / 2 + h - 1};
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
  d.font = &lv_font_montserrat_12;
  d.color = lv_color_hex(0xf0f0f0);
  d.align = LV_TEXT_ALIGN_CENTER;
  lv_area_t textArea = area;
  textArea.y1 += 1;
  lv_draw_label(layer, &d, &textArea);
  return area;
}

void drawScope(lv_event_t* e) {
  lv_layer_t* layer = lv_event_get_layer(e);
  lv_area_t box;
  lv_obj_get_coords(s_scope, &box);
  const int cx = box.x1 + kScope / 2, cy = box.y1 + kScope / 2;

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
    lv_draw_arc(layer, &ring);
    rangeText(buf, sizeof buf, viewNm() * i / 3, "");
    drawLabel(layer, buf, &lv_font_montserrat_12, kRingText, cx + 3, cy - ring.radius + 1, 40);
  }
  lv_draw_line_dsc_t cross;
  lv_draw_line_dsc_init(&cross);
  cross.color = kRing;
  cross.width = 1;
  cross.p1 = {static_cast<lv_value_precise_t>(cx - kScopeR), static_cast<lv_value_precise_t>(cy)};
  cross.p2 = {static_cast<lv_value_precise_t>(cx + kScopeR), static_cast<lv_value_precise_t>(cy)};
  lv_draw_line(layer, &cross);
  cross.p1 = {static_cast<lv_value_precise_t>(cx), static_cast<lv_value_precise_t>(cy - kScopeR)};
  cross.p2 = {static_cast<lv_value_precise_t>(cx), static_cast<lv_value_precise_t>(cy + kScopeR)};
  lv_draw_line(layer, &cross);
  // Inside the outer ring, opposite its distance label, so the window's edge can't cut it.
  drawLabel(layer, "N", &lv_font_montserrat_14, kRingText, cx - 14, cy - kScopeR + 1, 20);

  const float limit = viewNm() * kVisibleR / kScopeR;
  lv_draw_line_dsc_t trail;
  lv_draw_line_dsc_init(&trail);
  trail.width = 1;
  trail.opa = LV_OPA_50;
  for (const Aircraft& a : s.snap->aircraft) {
    if (!drawnSingly(a)) continue;
    auto it = s.trails.find(a.hex);
    if (it == s.trails.end() || it->second.size() < 2) continue;
    trail.color = altColor(a);
    const auto& pts = it->second;
    for (size_t i = 1; i < pts.size(); ++i) {
      float x0, y0, x1, y1;
      toScope(pts[i - 1].eastNm, pts[i - 1].northNm, x0, y0);
      toScope(pts[i].eastNm, pts[i].northNm, x1, y1);
      if (std::hypot(x0, y0) > kVisibleR || std::hypot(x1, y1) > kVisibleR) continue;
      trail.p1 = pt(cx + x0, cy + y0);
      trail.p2 = pt(cx + x1, cy + y1);
      lv_draw_line(layer, &trail);
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
  for (const Cluster& c : clusters) taken.push_back(drawCluster(layer, cx + c.x, cy + c.y, c.count));

  lv_draw_triangle_dsc_t tri;
  lv_draw_triangle_dsc_init(&tri);
  tri.bg_opa = LV_OPA_COVER;
  // Farthest first so near (more relevant) aircraft draw on top.
  for (auto it = s.snap->aircraft.rbegin(); it != s.snap->aircraft.rend(); ++it) {
    const Aircraft& a = *it;
    if (a.distNm > limit || !drawnSingly(a)) continue;
    float east, north, x, y;
    position(a, east, north);
    toScope(east, north, x, y);
    const float px = cx + x, py = cy + y;
    const lv_color_t c = altColor(a);
    if (a.hasTrack && !a.onGround) {
      const float t = a.track * geo::kDeg;
      auto at = [&](float ang, float len) { return pt(px + len * std::sin(ang), py - len * std::cos(ang)); };
      tri.bg_color = c;
      tri.p[0] = at(t, 9);
      tri.p[1] = at(t + 2.5f, 7);
      tri.p[2] = at(t - 2.5f, 7);
      lv_draw_triangle(layer, &tri);
    } else {
      drawRing(layer, px, py, a.onGround ? 2 : 4, a.onGround ? 3 : 4, c);
    }
    if (a.hex == s.selected) drawRing(layer, px, py, 13, 2, kSelect);
  }

  // Callouts: the selected aircraft, then nearest first, each in the first of four
  // spots around its marker that is clear of labels already placed. Aircraft that
  // find no spot stay unlabelled (still tappable).
  std::vector<const Aircraft*> order;
  if (const Aircraft* sel = findSelected()) order.push_back(sel);
  for (const Aircraft& a : s.snap->aircraft) {
    if (a.hex != s.selected) order.push_back(&a);
  }
  for (const Aircraft* a : order) {
    if (a->distNm > limit || !drawnSingly(*a)) continue;
    float east, north, x, y;
    position(*a, east, north);
    toScope(east, north, x, y);
    const int px = cx + std::lround(x), py = cy + std::lround(y);
    const std::string alt = altText(*a);
    snprintf(buf, sizeof buf, alt.empty() ? "%s" : "%s\n%s", a->label().c_str(), alt.c_str());
    lv_point_t size;
    lv_text_get_size(&size, buf, &lv_font_montserrat_12, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    const int w = size.x + 2, h = size.y;
    const lv_point_t spots[] = {{px + 9, py - h / 2}, {px - 9 - w, py - h / 2}, {px - w / 2, py - 10 - h},
                                {px - w / 2, py + 10}};
    for (const lv_point_t& sp : spots) {
      const lv_area_t area = {sp.x, sp.y, sp.x + w - 1, sp.y + h - 1};
      if (!inWindow(area, cx, cy) || std::any_of(taken.begin(), taken.end(), [&](const lv_area_t& t) { return overlaps(t, area); }))
        continue;
      drawLabel(layer, buf, &lv_font_montserrat_12, a->hex == s.selected ? kSelect : altColor(*a), sp.x, sp.y, w);
      taken.push_back(area);
      break;
    }
  }

  // Home.
  lv_draw_arc_dsc_t home;
  lv_draw_arc_dsc_init(&home);
  home.color = kText;
  home.width = 2;
  home.radius = 3;
  home.start_angle = 0;
  home.end_angle = 360;
  home.center = {cx, cy};
  lv_draw_arc(layer, &home);
}

const Aircraft* findSelected() {
  for (const Aircraft& a : s.snap->aircraft) {
    if (a.hex == s.selected) return &a;
  }
  return nullptr;
}

void refreshDetail() {
  const Aircraft* a = findSelected();
  if (!a) {
    s.selected.clear();
    lv_obj_add_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_list, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_add_flag(s_list, LV_OBJ_FLAG_HIDDEN);
  lv_obj_remove_flag(s_detail, LV_OBJ_FLAG_HIDDEN);
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
    const char* arrow = a->vrateFpm > 300 ? " " LV_SYMBOL_UP : (a->vrateFpm < -300 ? " " LV_SYMBOL_DOWN : "");
    snprintf(line, sizeof line, "%d ft%s\n", a->altFt, arrow);
    body += line;
    if (std::abs(a->vrateFpm) > 300) {
      snprintf(line, sizeof line, "%+d fpm\n", a->vrateFpm);
      body += line;
    }
  }
  if (a->gsKt >= 0) {
    snprintf(line, sizeof line, "%.0f kt", a->gsKt);
    body += line;
    if (a->hasTrack) {
      snprintf(line, sizeof line, "  trk %03.0f", a->track);
      body += line;
    }
    body += "\n";
  }
  snprintf(line, sizeof line, "%.1f nm  brg %03.0f\n", a->distNm, a->bearing);
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
  lv_obj_invalidate(s_scope);
}

void onDetailClicked(lv_event_t*) {
  s.selected.clear();
  refreshDetail();
  lv_obj_invalidate(s_scope);
}

void rebuildList() {
  s.rowHex.clear();
  // Airborne first (nearest first, as the snapshot is sorted), then parked.
  std::vector<const Aircraft*> rows;
  for (const Aircraft& a : s.snap->aircraft) {
    if (!a.onGround) rows.push_back(&a);
  }
  if (s.ground != Ground::Hide) {
    for (const Aircraft& a : s.snap->aircraft) {
      if (a.onGround) rows.push_back(&a);
    }
  }
  for (int i = 0; i < kMaxRows; ++i) {
    const Row& r = s_rows[i];
    if (i >= static_cast<int>(rows.size())) {
      lv_obj_add_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    const Aircraft& a = *rows[i];
    lv_label_set_text(r.name, a.label().c_str());
    lv_obj_set_style_text_color(r.name, altColor(a), 0);
    lv_label_set_text(r.alt, altText(a).c_str());
    char dist[12];
    rangeText(dist, sizeof dist, a.distNm, "");
    lv_label_set_text(r.dist, dist);
    lv_obj_remove_flag(r.obj, LV_OBJ_FLAG_HIDDEN);
    s.rowHex.push_back(a.hex);
  }
}

lv_obj_t* rowLabel(lv_obj_t* row, const lv_font_t* font, lv_color_t color, int x, int w, lv_text_align_t align) {
  lv_obj_t* l = lv_label_create(row);
  lv_obj_set_size(l, w, lv_font_get_line_height(font));
  lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
  lv_label_set_text(l, "");
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_obj_set_style_text_align(l, align, 0);
  lv_obj_align(l, LV_ALIGN_LEFT_MID, x, 0);
  return l;
}

// One line per aircraft: callsign, altitude, distance in nm. Type and the rest
// are in the details.
void createRows() {
  for (int i = 0; i < kMaxRows; ++i) {
    lv_obj_t* row = lv_obj_create(s_list);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), kRowH);
    lv_obj_set_style_pad_hor(row, 3, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(row, kDivider, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_bg_color(row, kButtonPressed, LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_STATE_PRESSED);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(row, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_HIDDEN));
    lv_obj_add_event_cb(row, onRowClicked, LV_EVENT_CLICKED, reinterpret_cast<void*>(static_cast<uintptr_t>(i)));
    const int w = kInnerW - 6;
    s_rows[i] = {row, rowLabel(row, &lv_font_montserrat_14, kText, 0, 74, LV_TEXT_ALIGN_LEFT),
                 rowLabel(row, &lv_font_montserrat_12, kText, 74, 40, LV_TEXT_ALIGN_RIGHT),
                 rowLabel(row, &lv_font_montserrat_12, kDim, 114, w - 114, LV_TEXT_ALIGN_RIGHT)};
  }
}

void updateRange() {
  char buf[16];
  rangeText(buf, sizeof buf, viewNm(), " nm");
  lv_label_set_text(s_range, buf);
  lv_obj_set_state(s_zoomOut, LV_STATE_DISABLED, s.zoom == 0);
  lv_obj_set_state(s_zoomIn, LV_STATE_DISABLED, s.zoom == kZoomLevels - 1);
}

void setZoom(int zoom) {
  s.zoom = std::clamp(zoom, 0, kZoomLevels - 1);
  updateRange();
  lv_obj_invalidate(s_scope);
}

void onZoomClicked(lv_event_t* e) {
  setZoom(s.zoom + static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e))));
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
  for (const Aircraft& a : s.snap->aircraft) {
    // A tap on a ground badge falls through to zoom, which splits the crowd.
    if (!drawnSingly(a)) continue;
    float east, north, x, y;
    position(a, east, north);
    toScope(east, north, x, y);
    const float d = std::hypot(x - tx, y - ty);
    if (d < bestD) {
      bestD = d;
      best = &a;
    }
  }
  if (best) {
    s.selected = best->hex;
  } else {
    s.zoom = (s.zoom + 1) % kZoomLevels;
    updateRange();
  }
  refreshDetail();
  lv_obj_invalidate(s_scope);
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
    snprintf(buf, sizeof buf, "%d in the air\n%s%ds ago", flyingNow(), s.error.empty() ? "" : "STALE ", age);
  }
  lv_label_set_text(s_status, buf);
  lv_obj_set_style_text_color(s_status, s.error.empty() || s.snap->fetchedUs == 0 ? kDim : kEmergency, 0);
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
  bool homeDirty, groundChanged;
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
  }
  if (homeDirty) {
    s.trails.clear();
    updateRange();
    lv_obj_invalidate(s_scope);
  }
  if (groundChanged) {
    const Aircraft* sel = findSelected();
    if (sel && sel->onGround && s.ground == Ground::Hide) s.selected.clear();
    rebuildList();
    refreshDetail();
    lv_obj_invalidate(s_scope);
  }
  s.error = feed::lastError();
  const uint32_t gen = feed::generation();
  if (gen != s.generation) {
    s.generation = gen;
    s.snap = feed::latest();
    recordSeen();
    updateTrails();
    rebuildList();
    if (!s.selected.empty()) refreshDetail();
    lv_obj_invalidate(s_scope);
  }
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

// "+" zooms in (a smaller range), as on a map.
lv_obj_t* zoomButton(lv_obj_t* parent, const char* symbol, int step, lv_align_t align) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_size(b, 44, 28);
  lv_obj_set_style_radius(b, 6, 0);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(b, kButton, 0);
  lv_obj_set_style_bg_color(b, kButtonPressed, LV_STATE_PRESSED);
  lv_obj_set_style_opa(b, LV_OPA_40, LV_STATE_DISABLED);
  lv_obj_align(b, align, 0, -2);
  lv_obj_add_event_cb(b, onZoomClicked, LV_EVENT_CLICKED, reinterpret_cast<void*>(static_cast<intptr_t>(step)));
  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, symbol);
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

  s_zoomOut = zoomButton(controls, LV_SYMBOL_MINUS, -1, LV_ALIGN_BOTTOM_LEFT);
  s_zoomIn = zoomButton(controls, LV_SYMBOL_PLUS, 1, LV_ALIGN_BOTTOM_RIGHT);
  s_range = lv_label_create(controls);
  lv_obj_set_style_text_font(s_range, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(s_range, kRingText, 0);
  lv_obj_align(s_range, LV_ALIGN_BOTTOM_MID, 0, -8);
  updateRange();

  lv_timer_create(tick, 250, nullptr);
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

void setNet(Net net, const std::string& message) {
  std::lock_guard<std::mutex> g(s_inMutex);
  s_inNet = net;
  s_inMessage = message;
}

}  // namespace ui
