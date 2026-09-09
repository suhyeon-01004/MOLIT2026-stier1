#include "morai_udp_bridge/control/control_watchdog.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_udp_bridge {

ControlWatchdog::ControlWatchdog(double timeout_sec, float safe_brake)
    : timeout_sec_(timeout_sec), safe_brake_(safe_brake) {
  if (!std::isfinite(timeout_sec_) || timeout_sec_ <= 0.0) {
    throw std::invalid_argument("command timeout must be finite and positive");
  }
  if (!std::isfinite(safe_brake_) || safe_brake_ < 0.0F ||
      safe_brake_ > 1.0F) {
    throw std::invalid_argument("safe brake must be finite and in [0, 1]");
  }
}

ControlInput ControlWatchdog::select(const ControlInput& latest,
                                     bool has_command,
                                     double receipt_age_sec) const {
  if (has_command && std::isfinite(receipt_age_sec) && receipt_age_sec >= 0.0 &&
      receipt_age_sec < timeout_sec_) {
    return latest;
  }
  return ControlInput{0.0F, safe_brake_, 0.0F};
}

}  // namespace morai_udp_bridge
