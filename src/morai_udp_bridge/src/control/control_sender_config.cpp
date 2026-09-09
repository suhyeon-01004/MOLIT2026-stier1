#include "morai_udp_bridge/control/control_sender_config.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_udp_bridge {

float validatedSafeBrakeCommand(double safe_brake_command) {
  if (!std::isfinite(safe_brake_command) || safe_brake_command < 0.0 ||
      safe_brake_command > 1.0) {
    throw std::invalid_argument("safe_brake_command must be finite and in [0, 1]");
  }
  return static_cast<float>(safe_brake_command);
}

ros::WallDuration controlSendPeriodFromRate(double send_rate_hz) {
  if (!std::isfinite(send_rate_hz) || send_rate_hz <= 0.0) {
    throw std::invalid_argument("send_rate_hz must be finite and positive");
  }

  const ros::WallDuration period(1.0 / send_rate_hz);
  if (period.toNSec() <= 0) {
    throw std::invalid_argument(
        "send_rate_hz produces a non-positive WallTimer period");
  }
  return period;
}

}  // namespace morai_udp_bridge
