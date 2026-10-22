#pragma once
#include <cmath>

namespace geo {

constexpr double kEarthRadiusNm = 3440.065;
constexpr double kDeg = M_PI / 180.0;

// Great-circle distance (haversine) and initial bearing, degrees true 0-360.
inline void rangeBearing(double lat1, double lon1, double lat2, double lon2, float& distNm, float& bearing) {
  const double p1 = lat1 * kDeg, p2 = lat2 * kDeg;
  const double dp = p2 - p1, dl = (lon2 - lon1) * kDeg;
  const double a = std::sin(dp / 2) * std::sin(dp / 2) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
  distNm = 2 * kEarthRadiusNm * std::atan2(std::sqrt(a), std::sqrt(1 - a));
  const double y = std::sin(dl) * std::cos(p2);
  const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
  bearing = std::fmod(std::atan2(y, x) / kDeg + 360.0, 360.0);
}

}  // namespace geo
