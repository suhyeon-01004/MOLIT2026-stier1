#pragma once

#include "morai_udp_bridge/control/control_protocol.hpp"

namespace morai_udp_bridge {

class ControlWatchdog {
 public:
  ControlWatchdog(double timeout_sec, float safe_brake);

  ControlInput select(const ControlInput& latest, bool has_command,
                      double receipt_age_sec) const;

 private:
  double timeout_sec_{0.0};
  float safe_brake_{0.0F};
};

}  // namespace morai_udp_bridge
