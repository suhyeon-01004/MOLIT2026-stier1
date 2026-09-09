#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <ros/names.h>
#include <ros/ros.h>

#include "morai_udp_bridge/ActuatorCommand.h"
#include "morai_udp_bridge/CompetitionVehicleStatus.h"
#include "morai_udp_bridge/GearCommand.h"
#include "morai_udp_bridge/control/control_sender_config.hpp"
#include "morai_udp_bridge/control/control_protocol.hpp"
#include "morai_udp_bridge/control/control_watchdog.hpp"
#include "morai_udp_bridge/control/gear_change_interlock.hpp"
#include "morai_udp_bridge/network/udp_sender.hpp"

namespace morai_udp_bridge {
namespace {

float finiteFloat(double value, const char* parameter) {
  if (!std::isfinite(value) ||
      value > static_cast<double>(std::numeric_limits<float>::max()) ||
      value < -static_cast<double>(std::numeric_limits<float>::max())) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be finite and representable as float");
  }
  return static_cast<float>(value);
}

void requirePositiveFinite(double value, const char* parameter) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be finite and positive");
  }
}

template <typename Value>
void loadOptionalParameter(const ros::NodeHandle& node, const char* name,
                           Value* value) {
  if (node.hasParam(name) && !node.getParam(name, *value)) {
    throw std::invalid_argument(std::string(name) +
                                " has an invalid parameter type");
  }
}

}  // namespace

class ControlSenderNode {
 public:
  ControlSenderNode() : node_(), private_node_("~") {
    std::string command_topic = "/control/actuator_command";
    std::string destination_ip = "127.0.0.1";
    int destination_port = 9093;
    double send_rate_hz = 50.0;
    double command_timeout_sec = 0.25;
    double safe_brake_command = 0.50;
    double maximum_steering_angle_deg = 40.0;
    double steering_sign = 1.0;
    int drive_gear = 4;
    bool gear_command_enabled = false;
    std::string gear_command_topic = "/control/gear_command";
    std::string vehicle_status_topic = "/vehicle/competition_status";
    double gear_change_maximum_abs_speed_mps = 0.1;
    double gear_change_status_timeout_sec = 0.25;
    double gear_change_minimum_brake_command = 0.5;
    double gear_change_maximum_accel_command = 0.05;

    loadOptionalParameter(private_node_, "command_topic", &command_topic);
    loadOptionalParameter(private_node_, "destination_ip", &destination_ip);
    loadOptionalParameter(private_node_, "destination_port", &destination_port);
    loadOptionalParameter(private_node_, "send_rate_hz", &send_rate_hz);
    loadOptionalParameter(private_node_, "command_timeout_sec",
                          &command_timeout_sec);
    loadOptionalParameter(private_node_, "safe_brake_command",
                          &safe_brake_command);
    loadOptionalParameter(private_node_, "maximum_steering_angle_deg",
                          &maximum_steering_angle_deg);
    loadOptionalParameter(private_node_, "steering_sign", &steering_sign);
    loadOptionalParameter(private_node_, "drive_gear", &drive_gear);
    loadOptionalParameter(private_node_, "gear_command_enabled",
                          &gear_command_enabled);
    loadOptionalParameter(private_node_, "gear_command_topic",
                          &gear_command_topic);
    loadOptionalParameter(private_node_, "vehicle_status_topic",
                          &vehicle_status_topic);
    loadOptionalParameter(private_node_,
                          "gear_change_maximum_abs_speed_mps",
                          &gear_change_maximum_abs_speed_mps);
    loadOptionalParameter(private_node_, "gear_change_status_timeout_sec",
                          &gear_change_status_timeout_sec);
    loadOptionalParameter(private_node_,
                          "gear_change_minimum_brake_command",
                          &gear_change_minimum_brake_command);
    loadOptionalParameter(private_node_,
                          "gear_change_maximum_accel_command",
                          &gear_change_maximum_accel_command);

    std::string name_error;
    if (command_topic.empty() || !ros::names::validate(command_topic, name_error)) {
      throw std::invalid_argument("command_topic is not a valid ROS name: " +
                                  name_error);
    }
    if (gear_command_topic.empty() ||
        !ros::names::validate(gear_command_topic, name_error)) {
      throw std::invalid_argument(
          "gear_command_topic is not a valid ROS name: " + name_error);
    }
    if (vehicle_status_topic.empty() ||
        !ros::names::validate(vehicle_status_topic, name_error)) {
      throw std::invalid_argument(
          "vehicle_status_topic is not a valid ROS name: " + name_error);
    }
    if (destination_port < 1 || destination_port > 65535) {
      throw std::invalid_argument("destination_port must be in 1..65535");
    }
    requirePositiveFinite(maximum_steering_angle_deg,
                          "maximum_steering_angle_deg");
    if (steering_sign != -1.0 && steering_sign != 1.0) {
      throw std::invalid_argument("steering_sign must be either -1 or 1");
    }
    if (drive_gear < 1 || drive_gear > 5) {
      throw std::invalid_argument("drive_gear must be in 1..5");
    }

    constexpr double kDegreesToRadians = 0.017453292519943295;
    protocol_config_.maximum_steering_angle_rad = finiteFloat(
        maximum_steering_angle_deg * kDegreesToRadians,
        "maximum_steering_angle_deg");
    protocol_config_.steering_sign = finiteFloat(steering_sign, "steering_sign");
    protocol_config_.drive_gear = static_cast<std::uint8_t>(drive_gear);
    gear_command_enabled_ = gear_command_enabled;
    gear_interlock_config_.maximum_abs_speed_mps =
        gear_change_maximum_abs_speed_mps;
    gear_interlock_config_.status_timeout_sec =
        gear_change_status_timeout_sec;
    gear_interlock_config_.minimum_brake_command =
        gear_change_minimum_brake_command;
    gear_interlock_config_.maximum_accel_command =
        gear_change_maximum_accel_command;
    validateGearChangeInterlockConfig(gear_interlock_config_);

    const float safe_brake = validatedSafeBrakeCommand(safe_brake_command);
    watchdog_.reset(new ControlWatchdog(command_timeout_sec, safe_brake));

    std::string protocol_error;
    if (!isValidControlInput(ControlInput{}, protocol_config_, &protocol_error)) {
      throw std::invalid_argument("invalid control protocol configuration: " +
                                  protocol_error);
    }

    sender_.reset(new UdpSender(
        destination_ip, static_cast<std::uint16_t>(destination_port)));
    command_subscriber_ = node_.subscribe(
        command_topic, 1U, &ControlSenderNode::onCommand, this);
    if (gear_command_enabled_) {
      vehicle_status_subscriber_ = node_.subscribe(
          vehicle_status_topic, 10U, &ControlSenderNode::onVehicleStatus,
          this);
      gear_command_subscriber_ = node_.subscribe(
          gear_command_topic, 1U, &ControlSenderNode::onGearCommand, this);
    }
    const ros::WallDuration send_period =
        controlSendPeriodFromRate(send_rate_hz);
    send_timer_ = node_.createWallTimer(send_period,
                                        &ControlSenderNode::onSendTimer, this);
    ROS_INFO(
        "MORAI control sender: target gear=%d, runtime gear command=%s",
        drive_gear, gear_command_enabled_ ? "enabled" : "disabled");
  }

  ControlSenderNode(const ControlSenderNode&) = delete;
  ControlSenderNode& operator=(const ControlSenderNode&) = delete;

 private:
  void onCommand(const ActuatorCommand::ConstPtr& message) {
    const ControlInput candidate{message->accel, message->brake,
                                 message->steering_angle_rad};
    std::string error;
    if (!isValidControlInput(candidate, protocol_config_, &error)) {
      ROS_WARN_THROTTLE(1.0, "discarding invalid actuator command: %s",
                        error.c_str());
      return;
    }

    latest_command_ = candidate;
    has_command_ = true;
    receipt_time_ = ros::WallTime::now();
  }

  void onVehicleStatus(
      const CompetitionVehicleStatus::ConstPtr& message) {
    if (!std::isfinite(message->velocity_x_mps)) {
      ROS_WARN_THROTTLE(
          1.0, "discarding Competition Vehicle Status with invalid velocity");
      return;
    }
    latest_vehicle_status_ = *message;
    has_vehicle_status_ = true;
    vehicle_status_receipt_time_ = ros::WallTime::now();
  }

  void onGearCommand(const GearCommand::ConstPtr& message) {
    const ros::WallTime now = ros::WallTime::now();
    const double command_age_sec =
        has_command_ ? (now - receipt_time_).toSec()
                     : std::numeric_limits<double>::infinity();
    const ControlInput selected =
        watchdog_->select(latest_command_, has_command_, command_age_sec);

    GearChangeContext context;
    context.current_gear = protocol_config_.drive_gear;
    context.requested_gear = message->gear;
    context.has_status = has_vehicle_status_;
    context.status_age_sec =
        has_vehicle_status_
            ? (now - vehicle_status_receipt_time_).toSec()
            : std::numeric_limits<double>::infinity();
    context.velocity_x_mps =
        has_vehicle_status_ ? latest_vehicle_status_.velocity_x_mps : 0.0;
    context.has_actuator_command = true;
    context.accel = selected.accel;
    context.brake = selected.brake;

    std::string reason;
    if (!canApplyGearChange(context, gear_interlock_config_, &reason)) {
      ROS_WARN("rejected gear request %u: %s",
               static_cast<unsigned int>(message->gear), reason.c_str());
      return;
    }
    protocol_config_.drive_gear = message->gear;
    ROS_INFO("accepted gear request: %u",
             static_cast<unsigned int>(message->gear));
  }

  void onSendTimer(const ros::WallTimerEvent&) {
    const double receipt_age_sec =
        has_command_ ? (ros::WallTime::now() - receipt_time_).toSec()
                     : std::numeric_limits<double>::infinity();
    const ControlInput selected =
        watchdog_->select(latest_command_, has_command_, receipt_age_sec);

    try {
      const MoraiControlPacket packet =
          encodeMoraiControlPacket(selected, protocol_config_);
      sender_->send(packet.data(), packet.size());
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "failed to send MORAI control datagram: %s",
                        error.what());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ControlProtocolConfig protocol_config_;
  GearChangeInterlockConfig gear_interlock_config_;
  bool gear_command_enabled_{false};
  std::unique_ptr<ControlWatchdog> watchdog_;
  std::unique_ptr<UdpSender> sender_;
  ControlInput latest_command_;
  bool has_command_{false};
  ros::WallTime receipt_time_;
  CompetitionVehicleStatus latest_vehicle_status_;
  bool has_vehicle_status_{false};
  ros::WallTime vehicle_status_receipt_time_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber vehicle_status_subscriber_;
  ros::Subscriber gear_command_subscriber_;
  ros::WallTimer send_timer_;
};

}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "control_sender_node");
  try {
    morai_udp_bridge::ControlSenderNode sender;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start MORAI control sender: %s", error.what());
    return 1;
  } catch (...) {
    ROS_FATAL("failed to start MORAI control sender: unknown exception");
    return 1;
  }
  return 0;
}
