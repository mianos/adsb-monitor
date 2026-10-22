#pragma once
#include <string>

// Landscape 480x320: radar scope on the left 320x320, clock, status and the
// nearest-aircraft list (or the selected aircraft's details) on the right.
// Tap a plane on the scope or a list row to select it; tap empty scope to zoom.
namespace ui {

enum class Net { Unprovisioned, Connecting, Online };
// Parked aircraft crowd airports: hide them, collapse each crowd into a count
// badge, or draw every one. Values match the "ground" setting.
enum class Ground { Hide = 0, Group = 1, Show = 2 };

void init(double homeLat, double homeLon, int radiusNm);
void setHome(double lat, double lon, int radiusNm);
void setGround(Ground mode);
// Thread-safe; applied on the next UI tick. A non-empty message is shown across
// the scope (provisioning, Wi-Fi reset).
void setNet(Net net, const std::string& message);

}  // namespace ui
