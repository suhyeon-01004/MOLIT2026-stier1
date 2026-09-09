#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

// A continuous transition avoids an abrupt controller switch near a threshold.
inline double moraiCurvatureWeightBlend(double curvature, double low, double high) {
  if (!std::isfinite(curvature) || !std::isfinite(low) || !std::isfinite(high) || low < 0. || high <= low)
    throw std::invalid_argument("Invalid curvature weight interval");
  const double t = std::max(0., std::min(1., (std::abs(curvature)-low)/(high-low)));
  return t*t*(3.-2.*t);
}
