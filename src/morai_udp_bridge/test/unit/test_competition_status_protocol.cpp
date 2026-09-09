#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "morai_udp_bridge/sensors/competition_status_protocol.hpp"

namespace morai_udp_bridge {
namespace {

constexpr std::size_t kPayloadOffset = 27U;
constexpr std::size_t kPayloadSize = 152U;
constexpr std::size_t kPacketSize = kPayloadOffset + kPayloadSize + 2U;
constexpr std::size_t kAttitudeOffset = kPayloadOffset + 62U;
constexpr std::size_t kHeadingOffset = kPayloadOffset + 70U;
constexpr std::size_t kVelocityXOffset = kPayloadOffset + 74U;

void writeUint32(std::uint32_t value, std::size_t offset,
                 std::vector<std::uint8_t>* packet) {
  (*packet)[offset] = static_cast<std::uint8_t>(value & 0xffU);
  (*packet)[offset + 1U] =
      static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  (*packet)[offset + 2U] =
      static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  (*packet)[offset + 3U] =
      static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void writeFloat(float value, std::size_t offset,
                std::vector<std::uint8_t>* packet) {
  std::uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "requires 32-bit float");
  std::memcpy(&bits, &value, sizeof(bits));
  writeUint32(bits, offset, packet);
}

std::vector<std::uint8_t> validPacket(float velocity_x_kph = 18.0F,
                                      float heading_deg = 63.9014F) {
  std::vector<std::uint8_t> packet(kPacketSize, 0U);
  constexpr std::array<char, 11U> kHeader{
      {'#', 'M', 'o', 'r', 'a', 'i', 'I', 'n', 'f', 'o', '$'}};
  std::copy(kHeader.begin(), kHeader.end(), packet.begin());
  writeUint32(static_cast<std::uint32_t>(kPayloadSize), 11U, &packet);
  packet[kPayloadOffset + 8U] = 2U;
  packet[kPayloadOffset + 9U] = 4U;
  writeFloat(-359.4935F, kAttitudeOffset, &packet);
  writeFloat(heading_deg, kHeadingOffset, &packet);
  writeFloat(velocity_x_kph, kVelocityXOffset, &packet);
  packet[kPacketSize - 2U] = '\r';
  packet[kPacketSize - 1U] = '\n';
  return packet;
}

TEST(CompetitionStatusProtocol, ReadsHeadingAndVelocity) {
  const std::vector<std::uint8_t> packet = validPacket(18.0F, 63.9014F);
  DecodedCompetitionVehicleStatus status;
  std::string error;

  ASSERT_TRUE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, &error))
      << error;
  EXPECT_EQ(status.control_mode, 2U);
  EXPECT_EQ(status.gear, 4U);
  EXPECT_NEAR(status.heading_deg, 63.9014F, 1.0e-5F);
  EXPECT_NEAR(status.velocity_x_mps, 5.0F, 1.0e-6F);
  EXPECT_TRUE(error.empty());
}

TEST(CompetitionStatusProtocol, PreservesSignedReverseVelocity) {
  const std::vector<std::uint8_t> packet = validPacket(-7.2F);
  DecodedCompetitionVehicleStatus status;

  ASSERT_TRUE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, nullptr));
  EXPECT_NEAR(status.velocity_x_mps, -2.0F, 1.0e-6F);
}

TEST(CompetitionStatusProtocol, RejectsOtherMoraiInfoLayouts) {
  std::vector<std::uint8_t> packet = validPacket();
  writeUint32(216U, 11U, &packet);
  DecodedCompetitionVehicleStatus status;
  std::string error;

  EXPECT_FALSE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, &error));
  EXPECT_NE(error.find("152"), std::string::npos);
}

TEST(CompetitionStatusProtocol, RejectsTruncatedOrBadTailPacket) {
  std::vector<std::uint8_t> packet = validPacket();
  DecodedCompetitionVehicleStatus status;

  EXPECT_FALSE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size() - 1U, &status, nullptr));

  packet.back() = 0U;
  EXPECT_FALSE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, nullptr));
}

TEST(CompetitionStatusProtocol, RejectsNonFiniteVelocity) {
  std::vector<std::uint8_t> packet = validPacket();
  writeFloat(std::numeric_limits<float>::quiet_NaN(),
             kVelocityXOffset, &packet);
  DecodedCompetitionVehicleStatus status;

  EXPECT_FALSE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, nullptr));
}

TEST(CompetitionStatusProtocol, RejectsNonFiniteHeading) {
  std::vector<std::uint8_t> packet = validPacket();
  writeFloat(std::numeric_limits<float>::quiet_NaN(),
             kHeadingOffset, &packet);
  DecodedCompetitionVehicleStatus status;

  EXPECT_FALSE(decodeCompetitionVehicleStatus(
      packet.data(), packet.size(), &status, nullptr));
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
