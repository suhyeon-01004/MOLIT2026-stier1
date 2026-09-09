#include "morai_udp_bridge/sensors/competition_status_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace morai_udp_bridge {
namespace {

constexpr std::size_t kHeaderSize = 11U;
constexpr std::size_t kDataLengthOffset = kHeaderSize;
constexpr std::size_t kPayloadOffset = 27U;
constexpr std::size_t kCompetitionPayloadSize = 152U;
constexpr std::size_t kTailSize = 2U;
constexpr std::size_t kControlModeOffset = kPayloadOffset + 8U;
constexpr std::size_t kGearOffset = kPayloadOffset + 9U;
constexpr std::size_t kHeadingOffset = kPayloadOffset + 70U;
constexpr std::size_t kVelocityXOffset = kPayloadOffset + 74U;
constexpr float kKilometresPerHourToMetresPerSecond = 1.0F / 3.6F;

void setError(const char* message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

std::uint32_t readUint32(const std::uint8_t* bytes) {
  return static_cast<std::uint32_t>(bytes[0U]) |
         (static_cast<std::uint32_t>(bytes[1U]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2U]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3U]) << 24U);
}

float readFloat(const std::uint8_t* bytes) {
  const std::uint32_t bits = readUint32(bytes);
  float value = 0.0F;
  static_assert(sizeof(bits) == sizeof(value),
                "MORAI protocol requires 32-bit floats");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

bool decodeCompetitionVehicleStatus(const std::uint8_t* packet,
                                    std::size_t packet_size,
                                    DecodedCompetitionVehicleStatus* status,
                                    std::string* error) {
  if (packet == nullptr || status == nullptr) {
    setError("packet and status must not be null", error);
    return false;
  }

  constexpr char kHeader[] = "#MoraiInfo$";
  if (packet_size < kPayloadOffset + kTailSize ||
      !std::equal(kHeader, kHeader + kHeaderSize, packet)) {
    setError("invalid Competition Vehicle Status header", error);
    return false;
  }

  const std::uint32_t data_length = readUint32(packet + kDataLengthOffset);
  if (data_length != kCompetitionPayloadSize) {
    setError("Competition Vehicle Status payload must be exactly 152 bytes",
             error);
    return false;
  }

  const std::size_t expected_size =
      kPayloadOffset + kCompetitionPayloadSize + kTailSize;
  if (packet_size != expected_size) {
    setError("truncated or oversized Competition Vehicle Status packet",
             error);
    return false;
  }
  if (packet[expected_size - 2U] != '\r' ||
      packet[expected_size - 1U] != '\n') {
    setError("invalid Competition Vehicle Status tail", error);
    return false;
  }

  const float heading_deg = readFloat(packet + kHeadingOffset);
  const float velocity_x_kph = readFloat(packet + kVelocityXOffset);
  const float velocity_x_mps =
      velocity_x_kph * kKilometresPerHourToMetresPerSecond;
  if (!std::isfinite(heading_deg) || !std::isfinite(velocity_x_mps)) {
    setError("Competition Vehicle Status heading or velocity_x is not finite",
             error);
    return false;
  }

  status->control_mode = packet[kControlModeOffset];
  status->gear = packet[kGearOffset];
  status->heading_deg = heading_deg;
  status->velocity_x_mps = velocity_x_mps;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace morai_udp_bridge
