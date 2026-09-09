#include "morai_udp_bridge/control/gear_change_interlock.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_udp_bridge {
namespace {

void requireFiniteRange(const char* name, double value, double minimum,
                        double maximum) {
  if (!std::isfinite(value) || value < minimum || value > maximum) {
    throw std::invalid_argument(std::string(name) + " must be finite and in [" +
                                std::to_string(minimum) + ", " +
                                std::to_string(maximum) + "]");
  }
}

void setReason(const char* value, std::string* reason) {
  if (reason != nullptr) {
    *reason = value;
  }
}

bool validGear(std::uint8_t gear) { return gear >= 1U && gear <= 5U; }

}  // namespace

void validateGearChangeInterlockConfig(
    const GearChangeInterlockConfig& config) {
  requireFiniteRange("maximum_abs_speed_mps", config.maximum_abs_speed_mps,
                     0.0, 1000.0);
  requireFiniteRange("status_timeout_sec", config.status_timeout_sec, 0.0,
                     3600.0);
  requireFiniteRange("minimum_brake_command", config.minimum_brake_command,
                     0.0, 1.0);
  requireFiniteRange("maximum_accel_command", config.maximum_accel_command,
                     0.0, 1.0);
}

bool canApplyGearChange(const GearChangeContext& context,
                        const GearChangeInterlockConfig& config,
                        std::string* reason) {
  validateGearChangeInterlockConfig(config);
  if (!validGear(context.current_gear) ||
      !validGear(context.requested_gear)) {
    setReason("gear must be in 1..5", reason);
    return false;
  }
  if (context.requested_gear == context.current_gear) {
    if (reason != nullptr) {
      reason->clear();
    }
    return true;
  }
  if (!context.has_status) {
    setReason("Competition Vehicle Status is unavailable", reason);
    return false;
  }
  if (!std::isfinite(context.status_age_sec) ||
      context.status_age_sec < 0.0 ||
      context.status_age_sec > config.status_timeout_sec) {
    setReason("Competition Vehicle Status is stale", reason);
    return false;
  }
  if (!std::isfinite(context.velocity_x_mps) ||
      std::fabs(context.velocity_x_mps) > config.maximum_abs_speed_mps) {
    setReason("vehicle must be stopped before changing gear", reason);
    return false;
  }
  if (!context.has_actuator_command) {
    setReason("actuator command is unavailable", reason);
    return false;
  }
  if (!std::isfinite(context.accel) ||
      context.accel > config.maximum_accel_command) {
    setReason("accel command is too high for a gear change", reason);
    return false;
  }
  if (!std::isfinite(context.brake) ||
      context.brake < config.minimum_brake_command) {
    setReason("brake command is too low for a gear change", reason);
    return false;
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

}  // namespace morai_udp_bridge
