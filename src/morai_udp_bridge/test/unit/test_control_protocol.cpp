#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include "morai_udp_bridge/control/control_protocol.hpp"

namespace morai_udp_bridge {
namespace {

float readFloat(const std::uint8_t* data) {
  float value = 0.0F;
  std::memcpy(&value, data, sizeof(value));
  return value;
}

TEST(ControlProtocol, EncodesCompleteMolitControlFixture) {
  const ControlProtocolConfig config;
  const ControlInput input{0.5F, 0.0F, -0.6981317008F};

  const auto actual = encodeMoraiControlPacket(input, config);
  const std::array<std::uint8_t, 55U> expected{{
      0x23U, 0x4dU, 0x6fU, 0x72U, 0x61U, 0x69U, 0x43U, 0x74U,
      0x72U, 0x6cU, 0x43U, 0x6dU, 0x64U, 0x24U,
      0x17U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x02U, 0x04U, 0x01U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x00U, 0x3fU,
      0x00U, 0x00U, 0x00U, 0x00U,
      0x00U, 0x00U, 0x80U, 0xbfU,
      0x0dU, 0x0aU,
  }};

  EXPECT_EQ(expected, actual);
}

TEST(ControlProtocol, ConvertsPhysicalSteeringAngleToNormalizedPacketValue) {
  ControlProtocolConfig config;
  ControlInput input{0.5F, 0.0F, config.maximum_steering_angle_rad};
  const auto packet = encodeMoraiControlPacket(input, config);
  EXPECT_FLOAT_EQ(1.0F, readFloat(packet.data() + 49U));
}

TEST(ControlProtocol, ConvertsMinusFortyDegreesToNegativeNormalizedValue) {
  const ControlInput input{0.0F, 0.0F, -0.6981317008F};
  const auto packet = encodeMoraiControlPacket(input, ControlProtocolConfig{});
  EXPECT_FLOAT_EQ(-1.0F, readFloat(packet.data() + 49U));
}

TEST(ControlProtocol, AppliesSteeringSignReversal) {
  ControlProtocolConfig config;
  config.steering_sign = -1.0F;
  const ControlInput input{0.0F, 0.0F, config.maximum_steering_angle_rad};
  const auto packet = encodeMoraiControlPacket(input, config);
  EXPECT_FLOAT_EQ(-1.0F, readFloat(packet.data() + 49U));
}

TEST(ControlProtocol, RejectsSimultaneousAccelAndBrake) {
  std::string error;
  EXPECT_FALSE(isValidControlInput(
      ControlInput{0.1F, 0.1F, 0.0F}, ControlProtocolConfig{}, &error));
  EXPECT_EQ("accel and brake must be mutually exclusive", error);
}

TEST(ControlProtocol, RejectsInvalidDriveGearConfiguration) {
  ControlProtocolConfig config;
  config.drive_gear = 0U;
  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{}, config),
               std::invalid_argument);
}

TEST(ControlProtocol, RejectsNonFiniteControlValues) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();

  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{nan, 0.0F, 0.0F},
                                        ControlProtocolConfig{}),
               std::invalid_argument);
  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{0.0F, infinity, 0.0F},
                                        ControlProtocolConfig{}),
               std::invalid_argument);
  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{0.0F, 0.0F, nan},
                                        ControlProtocolConfig{}),
               std::invalid_argument);
}

TEST(ControlProtocol, RejectsPedalValuesOutsideUnitRange) {
  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{-0.01F, 0.0F, 0.0F},
                                        ControlProtocolConfig{}),
               std::invalid_argument);
  EXPECT_THROW(encodeMoraiControlPacket(ControlInput{0.0F, 1.01F, 0.0F},
                                        ControlProtocolConfig{}),
               std::invalid_argument);
}

TEST(ControlProtocol, RejectsSteeringAnglesOutsideConfiguredRange) {
  const ControlProtocolConfig config;
  EXPECT_THROW(encodeMoraiControlPacket(
                   ControlInput{0.0F, 0.0F,
                                config.maximum_steering_angle_rad + 0.001F},
                   config),
               std::invalid_argument);
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
