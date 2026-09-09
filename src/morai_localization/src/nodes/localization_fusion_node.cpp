#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <morai_udp_bridge/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>

#include "morai_localization/estimation/velocity_estimator.hpp"
#include "morai_localization/estimation/position_estimator.hpp"
#include "morai_localization/orientation/imu_orientation.hpp"

namespace morai_localization {
namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kRadiansToDegrees = 57.29577951308232;

void requirePositive(const std::string& name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
}

struct YawChoice {
  bool valid{false};
  double yaw{0.0};
  ros::Time stamp;
  std::string source{"NONE"};
};

class LocalizationFusionNode {
 public:
  LocalizationFusionNode() : private_node_("~") {
    loadConfig();

    pose_publisher_ =
        node_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 20);
    odometry_publisher_ =
        node_.advertise<nav_msgs::Odometry>(odometry_topic_, 20);
    status_text_publisher_ =
        node_.advertise<std_msgs::String>(status_text_topic_, 10, true);
    gps_subscriber_ =
        node_.subscribe(gps_topic_, 20,
                        &LocalizationFusionNode::handleGps, this);
    imu_subscriber_ =
        node_.subscribe(imu_topic_, 50,
                        &LocalizationFusionNode::handleImu, this);
    competition_status_subscriber_ = node_.subscribe(
        competition_status_topic_, 50,
        &LocalizationFusionNode::handleCompetitionStatus, this);
    status_text_timer_ = node_.createWallTimer(
        ros::WallDuration(status_text_period_sec_),
        &LocalizationFusionNode::publishStatusText, this);
  }

 private:
  void loadConfig() {
    private_node_.param<std::string>(
        "gps_local_topic", gps_topic_, "/localization/gps/local_point");
    private_node_.param<std::string>(
        "imu_output_topic", imu_topic_, "/localization/imu/data");
    private_node_.param<std::string>(
        "competition_status_topic", competition_status_topic_,
        "/vehicle/competition_status");
    private_node_.param<std::string>(
        "pose_topic", pose_topic_, "/localization/pose");
    private_node_.param<std::string>(
        "odometry_topic", odometry_topic_, "/localization/odometry");
    private_node_.param<std::string>(
        "status_text_topic", status_text_topic_,
        "/localization/status_text");
    private_node_.param<std::string>("frame_id", map_frame_id_, "map");
    private_node_.param<std::string>("imu_frame_id", imu_frame_id_,
                                     "imu_link");
    private_node_.param<std::string>("base_frame_id", base_frame_id_,
                                     "base_link");
    private_node_.param<std::string>("tf_child_frame_id", tf_child_frame_id_,
                                     "base_footprint");
    private_node_.param("publish_tf", publish_tf_, true);
    private_node_.param("yaw_sign", yaw_sign_, 1.0);
    private_node_.param("yaw_offset_deg", yaw_offset_deg_, 0.0);
    private_node_.param("max_gps_age_sec", max_gps_age_sec_, 1.0);
    private_node_.param("max_primary_imu_age_sec",
                        max_primary_imu_age_sec_, 0.10);
    private_node_.param("max_imu_age_sec", max_imu_age_sec_, 0.25);
    private_node_.param("max_competition_status_age_sec",
                        max_competition_status_age_sec_, 0.25);
    private_node_.param("max_yaw_disagreement_deg",
                        max_yaw_disagreement_deg_, 2.0);
    private_node_.param("max_speed_disagreement_mps",
                        max_speed_disagreement_mps_, 2.0);
    private_node_.param("imu_recovery_samples", imu_recovery_samples_, 3);
    private_node_.param("status_text_period_sec", status_text_period_sec_,
                        0.5);
    private_node_.param("position_filter_time_constant_sec",
                        position_filter_time_constant_sec_, 0.0);
    if (!std::isfinite(position_filter_time_constant_sec_) ||
        position_filter_time_constant_sec_ < 0.0 ||
        position_filter_time_constant_sec_ > 1.0) {
      throw std::invalid_argument("position_filter_time_constant_sec must be in [0,1]");
    }

    VelocityEstimatorConfig config;
    private_node_.param("minimum_velocity_dt_sec", config.minimum_dt_sec,
                        0.005);
    private_node_.param("maximum_velocity_dt_sec", config.maximum_dt_sec,
                        0.25);
    private_node_.param("maximum_velocity_mps", config.maximum_speed_mps,
                        50.0);
    private_node_.param("velocity_filter_time_constant_sec",
                        config.filter_time_constant_sec, 0.10);

    if (gps_topic_.empty() || imu_topic_.empty() ||
        competition_status_topic_.empty() || pose_topic_.empty() ||
        odometry_topic_.empty() || status_text_topic_.empty() ||
        map_frame_id_.empty() || imu_frame_id_.empty() ||
        base_frame_id_.empty() || tf_child_frame_id_.empty()) {
      throw std::invalid_argument(
          "localization topic names and frame IDs must not be empty");
    }
    if (yaw_sign_ != 1.0 && yaw_sign_ != -1.0) {
      throw std::invalid_argument("yaw_sign must be either 1.0 or -1.0");
    }
    if (!std::isfinite(yaw_offset_deg_)) {
      throw std::invalid_argument("yaw_offset_deg must be finite");
    }
    requirePositive("max_gps_age_sec", max_gps_age_sec_);
    requirePositive("max_primary_imu_age_sec", max_primary_imu_age_sec_);
    requirePositive("max_imu_age_sec", max_imu_age_sec_);
    requirePositive("max_competition_status_age_sec",
                    max_competition_status_age_sec_);
    requirePositive("max_yaw_disagreement_deg", max_yaw_disagreement_deg_);
    requirePositive("max_speed_disagreement_mps",
                    max_speed_disagreement_mps_);
    requirePositive("status_text_period_sec", status_text_period_sec_);
    if (max_primary_imu_age_sec_ > max_imu_age_sec_) {
      throw std::invalid_argument(
          "max_primary_imu_age_sec must not exceed max_imu_age_sec");
    }
    if (imu_recovery_samples_ < 1) {
      throw std::invalid_argument("imu_recovery_samples must be positive");
    }
    velocity_estimator_ = std::make_unique<VelocityEstimator>(config);
  }

  static ros::Time messageStamp(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now() : stamp;
  }

  static double age(const ros::Time& now, const ros::Time& stamp) {
    return std::max(0.0, (now - stamp).toSec());
  }

  double correctedYaw(double raw_yaw) const {
    return normalizeAngle(yaw_sign_ * raw_yaw +
                          yaw_offset_deg_ * kDegreesToRadians);
  }

  void handleGps(const geometry_msgs::PointStamped::ConstPtr& point) {
    if (!point->header.frame_id.empty() &&
        point->header.frame_id != map_frame_id_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding GPS local point in frame '"
                   << point->header.frame_id << "' (expected '"
                   << map_frame_id_ << "')");
      return;
    }
    if (!std::isfinite(point->point.x) || !std::isfinite(point->point.y) ||
        !std::isfinite(point->point.z)) {
      ROS_WARN_THROTTLE(5.0, "discarding non-finite GPS local point");
      return;
    }

    geometry_msgs::PointStamped current = *point;
    current.header.stamp = messageStamp(current.header.stamp);
    current.header.frame_id = map_frame_id_;
    gps_ = current;
    has_gps_ = true;
    gps_pending_ = true;
    publishIfReady();
  }

  void handleImu(const sensor_msgs::Imu::ConstPtr& imu) {
    if (!imu->header.frame_id.empty() &&
        imu->header.frame_id != imu_frame_id_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding normalized IMU message in frame '"
                   << imu->header.frame_id << "' (expected '" << imu_frame_id_
                   << "')");
      return;
    }
    if (!isFinite(imu->angular_velocity) ||
        !isFinite(imu->linear_acceleration)) {
      ROS_WARN_THROTTLE(5.0, "discarding non-finite normalized IMU data");
      return;
    }

    try {
      imu_ = *imu;
      imu_.orientation = normalizeQuaternion(imu->orientation);
    } catch (const std::invalid_argument& error) {
      ROS_WARN_STREAM_THROTTLE(5.0, "discarding invalid normalized IMU: "
                                        << error.what());
      return;
    }
    imu_.header.stamp = messageStamp(imu_.header.stamp);
    imu_.header.frame_id = imu_frame_id_;
    has_imu_ = true;
    if (competition_yaw_active_) {
      imu_recovery_count_ =
          std::min(imu_recovery_count_ + 1, imu_recovery_samples_);
    }
    publishIfReady();
  }

  void handleCompetitionStatus(
      const morai_udp_bridge::CompetitionVehicleStatus::ConstPtr& status) {
    if (!std::isfinite(status->heading_deg) ||
        !std::isfinite(status->velocity_x_mps)) {
      ROS_WARN_THROTTLE(5.0,
                        "discarding non-finite Competition Vehicle Status");
      return;
    }
    competition_status_ = *status;
    competition_status_.header.stamp =
        messageStamp(competition_status_.header.stamp);
    has_competition_status_ = true;
    publishIfReady();
  }

  YawChoice selectYaw(const ros::Time& now) {
    const bool imu_primary =
        has_imu_ && age(now, imu_.header.stamp) <= max_primary_imu_age_sec_;
    const bool imu_usable =
        has_imu_ && age(now, imu_.header.stamp) <= max_imu_age_sec_;
    const bool status_usable =
        has_competition_status_ &&
        age(now, competition_status_.header.stamp) <=
            max_competition_status_age_sec_;

    if (competition_yaw_active_ && status_usable &&
        (!imu_primary || imu_recovery_count_ < imu_recovery_samples_)) {
      return competitionYawChoice();
    }
    if (imu_primary) {
      competition_yaw_active_ = false;
      imu_recovery_count_ = 0;
      YawChoice choice;
      choice.valid = true;
      choice.yaw = correctedYaw(yawFromQuaternion(imu_.orientation));
      choice.stamp = imu_.header.stamp;
      choice.source = "IMU";
      return choice;
    }
    if (status_usable) {
      competition_yaw_active_ = true;
      imu_recovery_count_ = 0;
      ROS_WARN_THROTTLE(
          2.0, "IMU yaw is late; using Competition Status heading fallback");
      return competitionYawChoice();
    }
    if (imu_usable) {
      YawChoice choice;
      choice.valid = true;
      choice.yaw = correctedYaw(yawFromQuaternion(imu_.orientation));
      choice.stamp = imu_.header.stamp;
      choice.source = "IMU_DEGRADED";
      return choice;
    }
    return YawChoice{};
  }

  YawChoice competitionYawChoice() const {
    YawChoice choice;
    choice.valid = true;
    choice.yaw = correctedYaw(
        static_cast<double>(competition_status_.heading_deg) *
        kDegreesToRadians);
    choice.stamp = competition_status_.header.stamp;
    choice.source = "COMPETITION";
    return choice;
  }

  void updateCrossChecks(const ros::Time& now,
                         const VelocityEstimate& gps_velocity) {
    yaw_delta_valid_ = false;
    speed_delta_valid_ = false;

    if (has_imu_ && has_competition_status_ &&
        age(now, imu_.header.stamp) <= max_primary_imu_age_sec_ &&
        age(now, competition_status_.header.stamp) <=
            max_competition_status_age_sec_ &&
        std::abs((imu_.header.stamp -
                  competition_status_.header.stamp).toSec()) <= 0.05) {
      const double imu_yaw =
          correctedYaw(yawFromQuaternion(imu_.orientation));
      const double status_yaw = competitionYawChoice().yaw;
      yaw_delta_deg_ =
          std::abs(normalizeAngle(imu_yaw - status_yaw)) * kRadiansToDegrees;
      yaw_delta_valid_ = true;
      if (yaw_delta_deg_ > max_yaw_disagreement_deg_) {
        ROS_WARN_STREAM_THROTTLE(
            2.0, "IMU/Competition yaw disagreement: " << yaw_delta_deg_
                                                        << " deg");
      }
    }

    if (gps_velocity.valid && has_competition_status_ &&
        age(now, competition_status_.header.stamp) <=
            max_competition_status_age_sec_) {
      speed_delta_mps_ = std::abs(
          gps_velocity.longitudinal_mps -
          static_cast<double>(competition_status_.velocity_x_mps));
      speed_delta_valid_ = true;
      if (speed_delta_mps_ > max_speed_disagreement_mps_) {
        ROS_WARN_STREAM_THROTTLE(
            2.0, "Competition/GPS longitudinal speed disagreement: "
                     << speed_delta_mps_ << " m/s");
      }
    }
  }

  void publishIfReady() {
    if (!gps_pending_ || !has_gps_) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (age(now, gps_.header.stamp) > max_gps_age_sec_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "waiting for current GPS (age="
                   << age(now, gps_.header.stamp) << " s)");
      return;
    }

    const YawChoice yaw_choice = selectYaw(now);
    if (!yaw_choice.valid) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "waiting for usable yaw (IMU and Competition Status are stale)");
      return;
    }

    const VelocityEstimate gps_velocity = velocity_estimator_->update(
        gps_.point.x, gps_.point.y, gps_.header.stamp.toSec(), yaw_choice.yaw);
    updateCrossChecks(now, gps_velocity);
    const geometry_msgs::Quaternion orientation =
        quaternionFromYaw(yaw_choice.yaw);
    const ros::Time stamp = gps_.header.stamp > yaw_choice.stamp
                                ? gps_.header.stamp
                                : yaw_choice.stamp;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position = gps_.point;
    const bool status_fresh =
        has_competition_status_ &&
        age(now, competition_status_.header.stamp) <=
            max_competition_status_age_sec_;
    const bool motion_aligned = status_fresh &&
        std::abs((gps_.header.stamp - competition_status_.header.stamp).toSec()) <= 0.05 &&
        std::abs((gps_.header.stamp - yaw_choice.stamp).toSec()) <= 0.05;
    const auto estimated_position = position_estimator_.update(
        gps_.point.x, gps_.point.y, gps_.header.stamp.toSec(),
        competition_status_.velocity_x_mps, yaw_choice.yaw, motion_aligned,
        position_filter_time_constant_sec_);
    pose.pose.position.x = estimated_position.x;
    pose.pose.position.y = estimated_position.y;
    position_correction_m_ = std::hypot(estimated_position.x - gps_.point.x,
                                      estimated_position.y - gps_.point.y);
    pose.pose.orientation = orientation;
    pose_publisher_.publish(pose);
    gps_pending_ = false;
    last_yaw_source_ = yaw_choice.source;

    if (status_fresh || gps_velocity.valid) {
      nav_msgs::Odometry odometry;
      odometry.header = pose.header;
      odometry.child_frame_id = base_frame_id_;
      odometry.pose.pose = pose.pose;
      if (status_fresh) {
        odometry.twist.twist.linear.x =
            competition_status_.velocity_x_mps;
        odometry.twist.twist.linear.y = 0.0;
        last_speed_source_ = "COMPETITION";
        last_speed_mps_ = competition_status_.velocity_x_mps;
      } else {
        odometry.twist.twist.linear.x = gps_velocity.longitudinal_mps;
        odometry.twist.twist.linear.y = gps_velocity.lateral_mps;
        last_speed_source_ = "GPS_FALLBACK";
        last_speed_mps_ = gps_velocity.longitudinal_mps;
      }
      if (has_imu_ && age(now, imu_.header.stamp) <= max_imu_age_sec_) {
        odometry.twist.twist.angular.z =
            yaw_sign_ * imu_.angular_velocity.z;
      }
      odometry_publisher_.publish(odometry);
    } else {
      last_speed_source_ = "NONE";
    }
    if (gps_velocity.valid) {
      last_gps_speed_valid_ = true;
      last_gps_speed_mps_ = gps_velocity.longitudinal_mps;
    }

    if (publish_tf_) {
      geometry_msgs::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = tf_child_frame_id_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = orientation;
      transform_broadcaster_.sendTransform(transform);
    }
  }

  static std::string formatAge(bool available, double value) {
    if (!available) {
      return "-";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(3) << value << "s";
    return output.str();
  }

  void publishStatusText(const ros::WallTimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double gps_age =
        has_gps_ ? age(now, gps_.header.stamp)
                 : std::numeric_limits<double>::infinity();
    const double imu_age =
        has_imu_ ? age(now, imu_.header.stamp)
                 : std::numeric_limits<double>::infinity();
    const double status_age =
        has_competition_status_
            ? age(now, competition_status_.header.stamp)
            : std::numeric_limits<double>::infinity();

    std::string state = "OK";
    if (!has_gps_ || gps_age > max_gps_age_sec_ ||
        last_yaw_source_ == "NONE") {
      state = "WAIT";
    } else if ((yaw_delta_valid_ &&
                yaw_delta_deg_ > max_yaw_disagreement_deg_) ||
               (speed_delta_valid_ &&
                speed_delta_mps_ > max_speed_disagreement_mps_)) {
      state = "WARN";
    } else if (last_yaw_source_ != "IMU" ||
               last_speed_source_ != "COMPETITION" ||
               imu_age > max_primary_imu_age_sec_ ||
               status_age > max_competition_status_age_sec_) {
      state = "DEGRADED";
    }

    std::ostringstream output;
    output << '[' << state << "] yaw=" << last_yaw_source_
           << " speed=" << last_speed_source_;
    if (last_speed_source_ != "NONE") {
      output << '(' << std::fixed << std::setprecision(2)
             << last_speed_mps_ << "m/s)";
    }
    output << " ages[gps=" << formatAge(has_gps_, gps_age)
           << " imu=" << formatAge(has_imu_, imu_age)
           << " status="
           << formatAge(has_competition_status_, status_age) << ']';
    if (yaw_delta_valid_) {
      output << " dyaw=" << std::fixed << std::setprecision(3)
             << yaw_delta_deg_ << "deg";
    }
    if (speed_delta_valid_) {
      output << " dv=" << std::fixed << std::setprecision(2)
             << speed_delta_mps_ << "m/s";
    }
    if (last_gps_speed_valid_) {
      output << " gps_v=" << std::fixed << std::setprecision(2)
             << last_gps_speed_mps_ << "m/s";
    }
    if (has_competition_status_) {
      output << " mode="
             << static_cast<unsigned int>(competition_status_.control_mode)
             << " gear="
             << static_cast<unsigned int>(competition_status_.gear);
    }

    output << " pos=" << (position_estimator_.corrected() ? "GPS+MOTION" : "GPS_RAW")
           << " correction=" << std::fixed << std::setprecision(3)
           << position_correction_m_ << "m";
    std_msgs::String message;
    message.data = output.str();
    status_text_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string gps_topic_;
  std::string imu_topic_;
  std::string competition_status_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string status_text_topic_;
  std::string map_frame_id_;
  std::string imu_frame_id_;
  std::string base_frame_id_;
  std::string tf_child_frame_id_;
  bool publish_tf_ = true;
  double yaw_sign_ = 1.0;
  double yaw_offset_deg_ = 0.0;
  double max_gps_age_sec_ = 1.0;
  double max_primary_imu_age_sec_ = 0.10;
  double max_imu_age_sec_ = 0.25;
  double max_competition_status_age_sec_ = 0.25;
  double max_yaw_disagreement_deg_ = 2.0;
  double max_speed_disagreement_mps_ = 2.0;
  int imu_recovery_samples_ = 3;
  double status_text_period_sec_ = 0.5;
  double position_filter_time_constant_sec_ = 0.0;
  double position_correction_m_ = 0.0;
  PositionEstimator position_estimator_;
  std::unique_ptr<VelocityEstimator> velocity_estimator_;
  ros::Publisher pose_publisher_;
  ros::Publisher odometry_publisher_;
  ros::Publisher status_text_publisher_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber competition_status_subscriber_;
  ros::WallTimer status_text_timer_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  geometry_msgs::PointStamped gps_;
  sensor_msgs::Imu imu_;
  morai_udp_bridge::CompetitionVehicleStatus competition_status_;
  bool has_gps_ = false;
  bool has_imu_ = false;
  bool has_competition_status_ = false;
  bool gps_pending_ = false;
  bool competition_yaw_active_ = false;
  int imu_recovery_count_ = 0;
  std::string last_yaw_source_{"NONE"};
  std::string last_speed_source_{"NONE"};
  double last_speed_mps_ = 0.0;
  bool last_gps_speed_valid_ = false;
  double last_gps_speed_mps_ = 0.0;
  bool yaw_delta_valid_ = false;
  double yaw_delta_deg_ = 0.0;
  bool speed_delta_valid_ = false;
  double speed_delta_mps_ = 0.0;
};

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_fusion");
  try {
    morai_localization::LocalizationFusionNode fusion;
    ROS_INFO(
        "Localization: IMU yaw primary, Competition heading fallback, "
        "Competition velocity primary, GPS velocity cross-check");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start localization fusion: "
                     << error.what());
    return 1;
  }
  return 0;
}
