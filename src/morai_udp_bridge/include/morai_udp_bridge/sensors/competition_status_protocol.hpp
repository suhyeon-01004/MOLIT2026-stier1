#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace morai_udp_bridge {

struct DecodedCompetitionVehicleStatus {
  std::uint8_t control_mode{0U};
  std::uint8_t gear{0U};
  float heading_deg{0.0F};
  float velocity_x_mps{0.0F};
};

bool decodeCompetitionVehicleStatus(const std::uint8_t* packet,
                                    std::size_t packet_size,
                                    DecodedCompetitionVehicleStatus* status,
                                    std::string* error);

}  // namespace morai_udp_bridge
