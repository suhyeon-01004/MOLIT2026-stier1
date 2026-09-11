#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/planning/wheel_corridor.hpp"
#include "morai_path_tracking/planning/map_clearance_guard.hpp"

namespace morai_path_tracking {
namespace {

TEST(MapClearanceGuard, RequiresSameGenerationFrameValidityAndFiniteComputation) {
  geometry_msgs::PointStamped map;
  const ros::Time stamp(10, 123);
  map.header.stamp=stamp; map.header.frame_id="map";
  map.point.x=.3; map.point.y=1.; map.point.z=2.;
  EXPECT_DOUBLE_EQ(.3, selectMapClearance(.1,map,stamp,"map"));
  EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,ros::Time(10,124),"map"));
  EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,stamp,"odom"));
  EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,ros::Time(),"map"));
  map.point.y=0.; EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,stamp,"map"));
  map.point.y=1.; map.point.x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,stamp,"map"));
  map.point.x=-.05; EXPECT_DOUBLE_EQ(-.05, selectMapClearance(.1,map,stamp,"map"));
  map.point.z=51.; EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,stamp,"map"));
  map.point.z=-1.; EXPECT_DOUBLE_EQ(.1, selectMapClearance(.1,map,stamp,"map"));
}

WheelCorridorConfig competitionCorridor() {
  WheelCorridorConfig config;
  config.lane_half_width_m = 1.30;
  config.vehicle_width_m = 1.892;
  config.wheelbase_m = 3.0;
  return config;
}

TEST(WheelCorridor, ReportsCenteredStraightWheelClearance) {
  const std::vector<Point2d> path = {
      {-5.0, 0.0}, {0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(0.946, result.maximum_wheel_offset_m, 1.0e-12);
  EXPECT_NEAR(0.354, result.minimum_clearance_m, 1.0e-12);
}

TEST(WheelCorridor, IncludesRearAndFrontOuterWheelsForLateralOffset) {
  const std::vector<Point2d> path = {
      {-5.0, 0.2}, {0.0, 0.2}, {5.0, 0.2}, {10.0, 0.2}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(1.146, result.maximum_wheel_offset_m, 1.0e-12);
  EXPECT_NEAR(0.154, result.minimum_clearance_m, 1.0e-12);
}

TEST(WheelCorridor, HeadingMisalignmentConsumesFrontWheelMargin) {
  const double yaw_rad = 0.1;
  const std::vector<Point2d> path = {
      {-5.0 * std::cos(yaw_rad), -5.0 * std::sin(yaw_rad)},
      {0.0, 0.0},
      {5.0 * std::cos(yaw_rad), 5.0 * std::sin(yaw_rad)},
      {10.0 * std::cos(yaw_rad), 10.0 * std::sin(yaw_rad)}};

  const WheelCorridorResult result =
      estimateWheelCorridor(path, competitionCorridor());

  ASSERT_TRUE(result.valid) << result.error;
  // 3*sin(0.1) + 0.946*cos(0.1): the front outer wheel is critical.
  EXPECT_NEAR(1.240774190293, result.maximum_wheel_offset_m, 1.0e-9);
  EXPECT_NEAR(0.059225809707, result.minimum_clearance_m, 1.0e-9);
}

TEST(WheelCorridor, RejectsInvalidDimensionsAndDegeneratePath) {
  WheelCorridorConfig config = competitionCorridor();
  config.lane_half_width_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {1.0, 0.0}}, config)
                   .valid);

  config = competitionCorridor();
  config.vehicle_width_m = 0.0;
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {1.0, 0.0}}, config)
                   .valid);

  config = competitionCorridor();
  EXPECT_FALSE(estimateWheelCorridor({{0.0, 0.0}, {0.0, 0.0}}, config)
                   .valid);
}

TEST(WheelCorridor, CurvedWheelLinearizationMirrorsWithRoadGeometry) {
  const auto left =
      linearizeWheelOffsets(0.12, 0.08, 0.11, competitionCorridor());
  const auto right =
      linearizeWheelOffsets(-0.12, -0.08, -0.11, competitionCorridor());
  const std::array<std::size_t, 4U> mirrored{{1U, 0U, 3U, 2U}};
  for (std::size_t index = 0U; index < left.size(); ++index) {
    ASSERT_TRUE(left[index].valid);
    ASSERT_TRUE(right[mirrored[index]].valid);
    EXPECT_NEAR(right[mirrored[index]].offset_m, -left[index].offset_m,
                1.0e-12);
    EXPECT_NEAR(right[mirrored[index]].lateral_error_gradient,
                left[index].lateral_error_gradient, 1.0e-12);
    EXPECT_NEAR(right[mirrored[index]].heading_error_gradient,
                left[index].heading_error_gradient, 1.0e-12);
  }
}

TEST(WheelCorridor, CurvedWheelLinearizationMatchesFiniteDifferences) {
  constexpr double kLateralError = 0.12;
  constexpr double kHeadingError = 0.08;
  constexpr double kCurvature = 0.11;
  constexpr double kEpsilon = 1.0e-6;
  const auto nominal = linearizeWheelOffsets(
      kLateralError, kHeadingError, kCurvature, competitionCorridor());
  const auto lateral_plus = linearizeWheelOffsets(
      kLateralError + kEpsilon, kHeadingError, kCurvature,
      competitionCorridor());
  const auto lateral_minus = linearizeWheelOffsets(
      kLateralError - kEpsilon, kHeadingError, kCurvature,
      competitionCorridor());
  const auto heading_plus = linearizeWheelOffsets(
      kLateralError, kHeadingError + kEpsilon, kCurvature,
      competitionCorridor());
  const auto heading_minus = linearizeWheelOffsets(
      kLateralError, kHeadingError - kEpsilon, kCurvature,
      competitionCorridor());
  for (std::size_t index = 0U; index < nominal.size(); ++index) {
    ASSERT_TRUE(nominal[index].valid);
    ASSERT_TRUE(lateral_plus[index].valid);
    ASSERT_TRUE(lateral_minus[index].valid);
    ASSERT_TRUE(heading_plus[index].valid);
    ASSERT_TRUE(heading_minus[index].valid);
    const double lateral_gradient =
        (lateral_plus[index].offset_m - lateral_minus[index].offset_m) /
        (2.0 * kEpsilon);
    const double heading_gradient =
        (heading_plus[index].offset_m - heading_minus[index].offset_m) /
        (2.0 * kEpsilon);
    EXPECT_NEAR(nominal[index].lateral_error_gradient, lateral_gradient,
                1.0e-7);
    EXPECT_NEAR(nominal[index].heading_error_gradient, heading_gradient,
                1.0e-7);
  }
}

TEST(WheelCorridor, ClearanceSpeedLimitIsContinuousAndBounded) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.18;
  config.recovery_full_m = 0.05;
  config.minimum_speed_mps = 10.0 / 3.6;

  const LaneClearanceSpeedLimit comfortable =
      computeLaneClearanceSpeedLimit(0.30, 58.0 / 3.6, config);
  const LaneClearanceSpeedLimit midpoint =
      computeLaneClearanceSpeedLimit(0.115, 58.0 / 3.6, config);
  const LaneClearanceSpeedLimit critical =
      computeLaneClearanceSpeedLimit(0.05, 58.0 / 3.6, config);

  ASSERT_TRUE(comfortable.valid) << comfortable.error;
  ASSERT_TRUE(midpoint.valid) << midpoint.error;
  ASSERT_TRUE(critical.valid) << critical.error;
  EXPECT_NEAR(0.0, comfortable.urgency, 1.0e-12);
  EXPECT_NEAR(58.0 / 3.6, comfortable.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(0.5, midpoint.urgency, 1.0e-12);
  EXPECT_NEAR(34.0 / 3.6, midpoint.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(1.0, critical.urgency, 1.0e-12);
  EXPECT_NEAR(10.0 / 3.6, critical.speed_limit_mps, 1.0e-12);
}

TEST(WheelCorridor, PositiveClearanceDoesNotUndercutCurveMinimumSpeed) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.18;
  config.recovery_full_m = 0.0;
  config.minimum_speed_mps = 10.0 / 3.6;

  const LaneClearanceSpeedLimit positive =
      computeLaneClearanceSpeedLimit(0.048, 59.0 / 3.6, config);
  const LaneClearanceSpeedLimit contact =
      computeLaneClearanceSpeedLimit(0.0, 59.0 / 3.6, config);

  ASSERT_TRUE(positive.valid) << positive.error;
  ASSERT_TRUE(contact.valid) << contact.error;
  EXPECT_GT(positive.speed_limit_mps, 15.0 / 3.6);
  EXPECT_NEAR(contact.speed_limit_mps, 10.0 / 3.6, 1.0e-12);
}

TEST(WheelCorridor, ClearanceSpeedLimitRejectsInvertedThresholds) {
  LaneClearanceSpeedConfig config;
  config.recovery_start_m = 0.05;
  config.recovery_full_m = 0.18;
  config.minimum_speed_mps = 10.0 / 3.6;

  EXPECT_FALSE(
      computeLaneClearanceSpeedLimit(0.1, 58.0 / 3.6, config).valid);
}

TEST(WheelCorridor, HeadingErrorSpeedLimitActsBeforeLargeHeadingLag) {
  HeadingErrorSpeedConfig config;
  config.recovery_start_rad = 4.0 * M_PI / 180.0;
  config.recovery_full_rad = 10.0 * M_PI / 180.0;
  config.minimum_speed_mps = 10.0 / 3.6;

  const HeadingErrorSpeedLimit aligned = computeHeadingErrorSpeedLimit(
      2.0 * M_PI / 180.0, 20.0 / 3.6, config);
  const HeadingErrorSpeedLimit midpoint = computeHeadingErrorSpeedLimit(
      7.0 * M_PI / 180.0, 20.0 / 3.6, config);
  const HeadingErrorSpeedLimit lagging = computeHeadingErrorSpeedLimit(
      -12.0 * M_PI / 180.0, 20.0 / 3.6, config);

  ASSERT_TRUE(aligned.valid) << aligned.error;
  ASSERT_TRUE(midpoint.valid) << midpoint.error;
  ASSERT_TRUE(lagging.valid) << lagging.error;
  EXPECT_NEAR(0.0, aligned.urgency, 1.0e-12);
  EXPECT_NEAR(20.0 / 3.6, aligned.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(0.5, midpoint.urgency, 1.0e-12);
  EXPECT_NEAR(15.0 / 3.6, midpoint.speed_limit_mps, 1.0e-12);
  EXPECT_NEAR(1.0, lagging.urgency, 1.0e-12);
  EXPECT_NEAR(10.0 / 3.6, lagging.speed_limit_mps, 1.0e-12);
}

TEST(WheelCorridor, HeadingErrorSpeedLimitRejectsInvalidThresholds) {
  HeadingErrorSpeedConfig config;
  config.recovery_start_rad = 0.2;
  config.recovery_full_rad = 0.1;
  config.minimum_speed_mps = 10.0 / 3.6;

  EXPECT_FALSE(computeHeadingErrorSpeedLimit(0.1, 20.0 / 3.6, config)
                   .valid);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
