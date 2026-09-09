#pragma once

#include <cmath>
#include <stdexcept>

namespace morai_localization {

// Short-horizon wheel-speed/heading prediction, continuously GPS anchored.
// It never consults the path or applies a desired lateral/path offset.
// ponytail: no lateral-slip state; use a validated state estimator if sustained
// sideslip matters. This short-horizon dequantizer is not outage localization.
class PositionEstimator {
 public:
  struct Point { double x; double y; };

  Point update(double x, double y, double stamp, double speed, double yaw,
               bool motion_valid, double time_constant) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(stamp) ||
        !std::isfinite(time_constant) || time_constant < 0.0) {
      initialized_ = false;
      throw std::invalid_argument("invalid position estimator measurement/config");
    }
    const bool usable = motion_valid && std::isfinite(speed) &&
                        std::isfinite(yaw) && std::abs(speed) <= 50.0;
    const double vx = usable ? speed * std::cos(yaw) : 0.0;
    const double vy = usable ? speed * std::sin(yaw) : 0.0;
    const double dt = stamp - stamp_;
    Point result{x, y};
    corrected_ = false;
    if (initialized_ && usable && time_constant > 0.0 && dt > 0.0 &&
        dt <= 0.25 && std::abs(speed) >= 0.1) {
      const Point predicted{position_.x + 0.5 * (vx_ + vx) * dt,
                            position_.y + 0.5 * (vy_ + vy) * dt};
      const double residual_x = x - predicted.x;
      const double residual_y = y - predicted.y;
      // Re-anchor on teleport/reinitialization or an inconsistent motion model.
      if (std::hypot(residual_x, residual_y) <= 0.75) {
        const double alpha = -std::expm1(-dt / time_constant);
        result = {predicted.x + alpha * residual_x,
                  predicted.y + alpha * residual_y};
        corrected_ = true;
      }
    }
    position_ = result;
    stamp_ = stamp;
    vx_ = vx;
    vy_ = vy;
    initialized_ = usable;
    return result;
  }

  bool corrected() const { return corrected_; }

 private:
  Point position_{0.0, 0.0};
  double stamp_{0.0}, vx_{0.0}, vy_{0.0};
  bool initialized_{false}, corrected_{false};
};

}  // namespace morai_localization
