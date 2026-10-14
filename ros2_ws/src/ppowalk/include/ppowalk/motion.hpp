#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace ppowalk
{

// Tilt of the body z axis from vertical (degrees), from the gravity direction
// in the body frame (getup::projected_gravity): 0 when upright, 180 upside down.
inline double tilt_deg(double gx, double gy, double gz)
{
  const double norm = std::sqrt(gx * gx + gy * gy + gz * gz);
  if (norm < 1e-9) {
    return 180.0;
  }
  const double c = std::clamp(-gz / norm, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

// Duration of a cosine-profile move whose peak joint speed is <= max_vel
// (peak = pi/2 * average speed), never shorter than min_duration.
inline double safe_move_duration(
  const std::vector<double> & from, const std::vector<double> & to, double max_vel,
  double min_duration)
{
  double delta = 0.0;
  for (size_t i = 0; i < from.size() && i < to.size(); ++i) {
    delta = std::max(delta, std::abs(to[i] - from[i]));
  }
  return std::max(min_duration, 0.5 * M_PI * delta / std::max(max_vel, 1e-6));
}

// Cosine blend weight for s in [0, 1] (zero velocity at both ends).
inline double smooth_alpha(double s)
{
  s = std::clamp(s, 0.0, 1.0);
  return 0.5 - 0.5 * std::cos(M_PI * s);
}

inline std::vector<double> blend(
  const std::vector<double> & from, const std::vector<double> & to, double alpha)
{
  std::vector<double> out(from.size());
  for (size_t i = 0; i < from.size(); ++i) {
    out[i] = from[i] + alpha * (to[i] - from[i]);
  }
  return out;
}

}  // namespace ppowalk
