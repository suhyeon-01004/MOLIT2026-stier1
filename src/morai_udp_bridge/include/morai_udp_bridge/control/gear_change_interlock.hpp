#pragma once

#include <cstdint>
#include <string>

namespace morai_udp_bridge {

struct GearChangeInterlockConfig {
  double maximum_abs_speed_mps{0.1};
  double status_timeout_sec{0.25};
  double minimum_brake_command{0.5};
  double maximum_accel_command{0.05};
};

struct GearChangeContext {
  std::uint8_t current_gear{4U};
  std::uint8_t requested_gear{4U};
  bool has_status{false};
  double status_age_sec{0.0};
  double velocity_x_mps{0.0};
  bool has_actuator_command{false};
  double accel{0.0};
  double brake{0.0};
};

void validateGearChangeInterlockConfig(
    const GearChangeInterlockConfig& config);
bool canApplyGearChange(const GearChangeContext& context,
                        const GearChangeInterlockConfig& config,
                        std::string* reason);

}  // namespace morai_udp_bridge
