#include "morai_udp_bridge/control/control_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace morai_udp_bridge {
namespace {

void setError(const char* message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

void writeUint32(std::uint32_t value, std::size_t offset,
                 MoraiControlPacket* packet) {
  (*packet)[offset] = static_cast<std::uint8_t>(value & 0xffU);
  (*packet)[offset + 1U] =
      static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  (*packet)[offset + 2U] =
      static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  (*packet)[offset + 3U] =
      static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void writeFloat(float value, std::size_t offset, MoraiControlPacket* packet) {
  static_assert(sizeof(float) == sizeof(std::uint32_t),
                "MORAI protocol requires 32-bit floats");
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeUint32(bits, offset, packet);
}

bool isValidUnitPedal(float value, const char* name, std::string* error) {
  if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
    setError(name, error);
    return false;
  }
  return true;
}

}  // namespace

bool isValidControlInput(const ControlInput& input,
                         const ControlProtocolConfig& config,
                         std::string* error) {
  if (config.drive_gear < 1U || config.drive_gear > 5U) {
    setError("drive gear must be in 1..5", error);
    return false;
  }
  if (!std::isfinite(config.maximum_steering_angle_rad) ||
      config.maximum_steering_angle_rad <= 0.0F) {
    setError("maximum steering angle must be finite and positive", error);
    return false;
  }
  if (config.steering_sign != -1.0F && config.steering_sign != 1.0F) {
    setError("steering sign must be either -1 or 1", error);
    return false;
  }
  if (!isValidUnitPedal(input.accel, "accel must be finite and in [0, 1]",
                        error) ||
      !isValidUnitPedal(input.brake, "brake must be finite and in [0, 1]",
                        error)) {
    return false;
  }
  if (input.accel > 0.0F && input.brake > 0.0F) {
    setError("accel and brake must be mutually exclusive", error);
    return false;
  }
  if (!std::isfinite(input.steering_angle_rad) ||
      std::fabs(input.steering_angle_rad) >
          config.maximum_steering_angle_rad) {
    setError("steering angle must be finite and within configured range",
             error);
    return false;
  }
  return true;
}

MoraiControlPacket encodeMoraiControlPacket(
    const ControlInput& input, const ControlProtocolConfig& config) {
  std::string error;
  if (!isValidControlInput(input, config, &error)) {
    throw std::invalid_argument(error);
  }

  MoraiControlPacket packet{};
  constexpr char kHeader[] = "#MoraiCtrlCmd$";
  static_assert(sizeof(kHeader) - 1U == 14U,
                "MORAI control header must be 14 bytes");
  std::copy(kHeader, kHeader + 14U, packet.begin());
  writeUint32(23U, 14U, &packet);
  packet[30U] = 2U;
  packet[31U] = config.drive_gear;
  packet[32U] = 1U;
  writeFloat(0.0F, 33U, &packet);
  writeFloat(0.0F, 37U, &packet);
  writeFloat(input.accel, 41U, &packet);
  writeFloat(input.brake, 45U, &packet);
  const float normalized = config.steering_sign * input.steering_angle_rad /
                           config.maximum_steering_angle_rad;
  writeFloat(normalized, 49U, &packet);
  packet[53U] = '\r';
  packet[54U] = '\n';
  return packet;
}

}  // namespace morai_udp_bridge
