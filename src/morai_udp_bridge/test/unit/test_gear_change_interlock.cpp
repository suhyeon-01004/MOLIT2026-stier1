#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "morai_udp_bridge/control/gear_change_interlock.hpp"

namespace morai_udp_bridge {
namespace {

GearChangeInterlockConfig testConfig() {
  GearChangeInterlockConfig config;
  config.maximum_abs_speed_mps = 0.1;
  config.status_timeout_sec = 0.25;
  config.minimum_brake_command = 0.5;
  config.maximum_accel_command = 0.05;
  return config;
}

GearChangeContext safeContext() {
  GearChangeContext context;
  context.current_gear = 4U;
  context.requested_gear = 2U;
  context.has_status = true;
  context.status_age_sec = 0.05;
  context.velocity_x_mps = 0.0;
  context.has_actuator_command = true;
  context.accel = 0.0;
  context.brake = 0.6;
  return context;
}

TEST(GearChangeInterlock, AcceptsStoppedBrakedTransition) {
  std::string reason;
  EXPECT_TRUE(canApplyGearChange(safeContext(), testConfig(), &reason))
      << reason;
  EXPECT_TRUE(reason.empty());
}

TEST(GearChangeInterlock, SameGearIsIdempotentWithoutStatus) {
  GearChangeContext context;
  context.current_gear = 4U;
  context.requested_gear = 4U;

  EXPECT_TRUE(canApplyGearChange(context, testConfig(), nullptr));
}

TEST(GearChangeInterlock, RejectsInvalidGearAndUnsafeMotion) {
  GearChangeContext context = safeContext();
  context.requested_gear = 0U;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));

  context = safeContext();
  context.velocity_x_mps = -0.11;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));

  context = safeContext();
  context.status_age_sec = 0.251;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));
}

TEST(GearChangeInterlock, RejectsMissingStatusOrUnsafePedals) {
  GearChangeContext context = safeContext();
  context.has_status = false;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));

  context = safeContext();
  context.has_actuator_command = false;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));

  context = safeContext();
  context.accel = 0.051;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));

  context = safeContext();
  context.brake = 0.49;
  EXPECT_FALSE(canApplyGearChange(context, testConfig(), nullptr));
}

TEST(GearChangeInterlock, RejectsInvalidConfiguration) {
  GearChangeInterlockConfig config = testConfig();
  config.status_timeout_sec = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validateGearChangeInterlockConfig(config),
               std::invalid_argument);
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
