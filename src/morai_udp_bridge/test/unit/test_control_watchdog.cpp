#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "morai_udp_bridge/control/control_watchdog.hpp"

namespace morai_udp_bridge {
namespace {

TEST(ControlWatchdog, ReplacesMissingCommandWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);

  const ControlInput selected =
      watchdog.select(ControlInput{0.4F, 0.0F, 0.2F}, false, 0.0);

  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, PreservesFreshCommand) {
  ControlWatchdog watchdog(0.25, 0.5F);
  const ControlInput latest{0.4F, 0.0F, 0.2F};

  const ControlInput selected = watchdog.select(latest, true, 0.249);

  EXPECT_FLOAT_EQ(0.4F, selected.accel);
  EXPECT_FLOAT_EQ(0.0F, selected.brake);
  EXPECT_FLOAT_EQ(0.2F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, ReplacesCommandAtTimeoutWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);

  const ControlInput selected =
      watchdog.select(ControlInput{0.4F, 0.0F, 0.2F}, true, 0.25);

  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, ReplacesStaleCommandWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);
  const ControlInput selected =
      watchdog.select(ControlInput{0.4F, 0.0F, 0.2F}, true, 0.251);
  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, ReplacesNonFiniteReceiptAgeWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);

  const ControlInput selected = watchdog.select(
      ControlInput{0.4F, 0.0F, 0.2F}, true,
      std::numeric_limits<double>::quiet_NaN());

  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, ReplacesNegativeReceiptAgeWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);

  const ControlInput selected =
      watchdog.select(ControlInput{0.4F, 0.0F, 0.2F}, true, -0.001);

  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}

TEST(ControlWatchdog, RejectsInvalidSafeBrakeConfiguration) {
  EXPECT_THROW(ControlWatchdog(0.25, -0.01F), std::invalid_argument);
  EXPECT_THROW(ControlWatchdog(0.25, 1.01F), std::invalid_argument);
  EXPECT_THROW(
      ControlWatchdog(0.25, std::numeric_limits<float>::quiet_NaN()),
      std::invalid_argument);
}

TEST(ControlWatchdog, RejectsInvalidTimeoutConfiguration) {
  EXPECT_THROW(ControlWatchdog(0.0, 0.5F), std::invalid_argument);
  EXPECT_THROW(
      ControlWatchdog(std::numeric_limits<double>::infinity(), 0.5F),
      std::invalid_argument);
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
