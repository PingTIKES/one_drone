#pragma once
#include <cmath>
#include <stdexcept>

namespace plan_env {
inline double inflationRadius(double body_radius, double safety_margin) {
  if (!std::isfinite(body_radius) || body_radius <= 0.0 ||
      !std::isfinite(safety_margin) || safety_margin < 0.0)
    throw std::invalid_argument("body_radius must be positive; safety_margin must be nonnegative (metres)");
  // Canonicalize decimal metre inputs: 0.10 + 0.20 must not inflate to
  // an extra voxel through ceil(0.30000000000000004 / 0.15).
  return std::round((body_radius + safety_margin) * 1e9) / 1e9;
}
}  // namespace plan_env
