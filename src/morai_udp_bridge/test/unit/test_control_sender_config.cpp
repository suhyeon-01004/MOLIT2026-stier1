#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <ros/duration.h>

#include "morai_udp_bridge/control/control_sender_config.hpp"

namespace morai_udp_bridge {
namespace {

TEST(ControlSenderConfig, RejectsSafeBrakeValuesLostDuringFloatNarrowing) {
  const double just_above_one =
      std::nextafter(1.0, std::numeric_limits<double>::infinity());
  EXPECT_FLOAT_EQ(1.0F, static_cast<float>(just_above_one));
  EXPECT_TRUE(std::signbit(static_cast<float>(-1.0e-50)));

  EXPECT_THROW(validatedSafeBrakeCommand(-1.0e-50), std::invalid_argument);
  EXPECT_THROW(validatedSafeBrakeCommand(just_above_one),
               std::invalid_argument);
}

TEST(ControlSenderConfig, PreservesSafeBrakeEndpoints) {
  EXPECT_FLOAT_EQ(0.0F, validatedSafeBrakeCommand(0.0));
  EXPECT_FLOAT_EQ(1.0F, validatedSafeBrakeCommand(1.0));
}

TEST(ControlSenderConfig, RejectsRateWhoseWallPeriodRoundsToZero) {
  const double rate_hz = 2.0000001e9;
  const ros::WallDuration rounded_period(1.0 / rate_hz);
  ASSERT_EQ(0, rounded_period.toNSec());

  EXPECT_THROW(controlSendPeriodFromRate(rate_hz), std::invalid_argument);
}

TEST(ControlSenderConfig, ReturnsRepresentablePositiveWallPeriod) {
  const ros::WallDuration period = controlSendPeriodFromRate(1.0e9);

  EXPECT_EQ(1, period.toNSec());
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
