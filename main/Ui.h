#pragma once
#include <string>

// Landscape 480x320: a round radar scope on the left 320x320 (the case hides the
// corners), and on the right a scrolling nearest-aircraft list (or the selected
// aircraft's details, or settings) above the clock, status, range +/- and the
// settings button.
// Tap a plane on the scope or a list row to select it; tap empty scope to zoom.
namespace ui {

enum class Net { Unprovisioned, Connecting, Online };
// Parked aircraft crowd airports: hide them, collapse each crowd into a count
// badge, or draw every one. Values match the "ground" setting.
enum class Ground { Hide = 0, Group = 1, Show = 2 };

void init(double homeLat, double homeLon, int radiusNm);
void setHome(double lat, double lon, int radiusNm);
void setGround(Ground mode);
// Called on the UI task when the settings view changes the mode, so it can be saved.
void onGroundChosen(void (*cb)(Ground mode));
// Zoom level 0 (full range), 1 (half) or 2 (quarter); the "zoom" setting.
void setZoom(int level);
void onZoomChosen(void (*cb)(int level));
// The decorative rotating sweep line; the "sweep" setting.
void setSweep(bool on);
void onSweepChosen(void (*cb)(bool on));
// Metric (m, km, km/h) or aviation units (ft, nm, kt); the "units" setting.
void setUnits(bool metric);
void onUnitsChosen(void (*cb)(bool metric));
// Thread-safe; applied on the next UI tick. A non-empty message is shown across
// the scope (provisioning, Wi-Fi reset).
void setNet(Net net, const std::string& message);

}  // namespace ui
