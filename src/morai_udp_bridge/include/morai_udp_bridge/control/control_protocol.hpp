#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace morai_udp_bridge {

struct ControlInput {
  float accel{0.0F};
  float brake{0.0F};
  float steering_angle_rad{0.0F};
};

struct ControlProtocolConfig {
  float maximum_steering_angle_rad{0.6981317008F};
  float steering_sign{1.0F};
  std::uint8_t drive_gear{4U};
};

using MoraiControlPacket = std::array<std::uint8_t, 55U>;

bool isValidControlInput(const ControlInput& input,
                         const ControlProtocolConfig& config,
                         std::string* error);
MoraiControlPacket encodeMoraiControlPacket(
    const ControlInput& input, const ControlProtocolConfig& config);

}  // namespace morai_udp_bridge
