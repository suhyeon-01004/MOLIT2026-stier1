#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/names.h>
#include <ros/ros.h>

#include "morai_udp_bridge/CompetitionVehicleStatus.h"
#include "morai_udp_bridge/network/transport.hpp"
#include "morai_udp_bridge/sensors/competition_status_protocol.hpp"

namespace morai_udp_bridge {
namespace {

template <typename Value>
void loadOptionalParameter(const ros::NodeHandle& node, const char* name,
                           Value* value) {
  if (node.hasParam(name) && !node.getParam(name, *value)) {
    throw std::invalid_argument(std::string(name) +
                                " has an invalid parameter type");
  }
}

void requireRosName(const char* parameter, const std::string& value) {
  std::string error;
  if (value.empty() || !ros::names::validate(value, error)) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be a valid ROS name: " + error);
  }
}

void requirePositiveFinite(const char* parameter, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(parameter) +
                                " must be finite and positive");
  }
}

}  // namespace

class CompetitionVehicleStatusReceiverNode {
 public:
  CompetitionVehicleStatusReceiverNode() : private_node_("~") {
    std::string bind_ip = "0.0.0.0";
    std::string allowed_source_ip;
    int listen_port = 9094;
    int receive_buffer_bytes = 1048576;
    std::string status_topic = "/vehicle/competition_status";
    std::string diagnostics_topic = "/diagnostics";
    double stale_timeout_sec = 0.25;
    double maximum_publish_hz = 50.0;
    double diagnostics_period_sec = 1.0;

    loadOptionalParameter(private_node_, "bind_ip", &bind_ip);
    loadOptionalParameter(private_node_, "allowed_source_ip",
                          &allowed_source_ip);
    loadOptionalParameter(private_node_, "listen_port", &listen_port);
    loadOptionalParameter(private_node_, "receive_buffer_bytes",
                          &receive_buffer_bytes);
    loadOptionalParameter(private_node_, "status_topic", &status_topic);
    loadOptionalParameter(private_node_, "diagnostics_topic",
                          &diagnostics_topic);
    loadOptionalParameter(private_node_, "stale_timeout_sec",
                          &stale_timeout_sec);
    loadOptionalParameter(private_node_, "maximum_publish_hz",
                          &maximum_publish_hz);
    loadOptionalParameter(private_node_, "diagnostics_period_sec",
                          &diagnostics_period_sec);

    if (listen_port < 1 || listen_port > 65535) {
      throw std::invalid_argument("listen_port must be in 1..65535");
    }
    if (receive_buffer_bytes <= 0) {
      throw std::invalid_argument("receive_buffer_bytes must be positive");
    }
    requireRosName("status_topic", status_topic);
    requireRosName("diagnostics_topic", diagnostics_topic);
    requirePositiveFinite("stale_timeout_sec", stale_timeout_sec);
    requirePositiveFinite("maximum_publish_hz", maximum_publish_hz);
    requirePositiveFinite("diagnostics_period_sec", diagnostics_period_sec);

    status_publisher_ = node_.advertise<
        morai_udp_bridge::CompetitionVehicleStatus>(status_topic, 10U);
    diagnostics_publisher_ =
        node_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic,
                                                          1U);

    stats_.reset(new StreamStats(
        "competition_vehicle_status", bind_ip,
        static_cast<std::uint16_t>(listen_port), status_topic,
        stale_timeout_sec, maximum_publish_hz));
    worker_.reset(new UdpWorker(
        bind_ip, static_cast<std::uint16_t>(listen_port),
        [this](const std::uint8_t* bytes, std::size_t size) {
          onDatagram(bytes, size);
        },
        stats_, static_cast<std::size_t>(receive_buffer_bytes),
        allowed_source_ip));
    diagnostics_timer_ = node_.createWallTimer(
        ros::WallDuration(diagnostics_period_sec),
        &CompetitionVehicleStatusReceiverNode::onDiagnosticsTimer, this);
    worker_->start();
    ROS_INFO(
        "Competition Vehicle Status receiver: udp://%s:%d -> %s "
        "(heading in degrees, velocity_x converted from km/h to m/s)",
        bind_ip.c_str(), listen_port, status_topic.c_str());
  }

 private:
  void onDatagram(const std::uint8_t* bytes, std::size_t size) {
    DecodedCompetitionVehicleStatus decoded;
    std::string error;
    if (!decodeCompetitionVehicleStatus(bytes, size, &decoded, &error)) {
      stats_->parseError();
      ROS_WARN_THROTTLE(2.0,
                        "discarding invalid Competition Vehicle Status: %s",
                        error.c_str());
      return;
    }

    morai_udp_bridge::CompetitionVehicleStatus message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "base_link";
    message.control_mode = decoded.control_mode;
    message.gear = decoded.gear;
    message.heading_deg = decoded.heading_deg;
    message.velocity_x_mps = decoded.velocity_x_mps;
    status_publisher_.publish(message);
    stats_->published();
  }

  void onDiagnosticsTimer(const ros::WallTimerEvent&) {
    diagnostic_msgs::DiagnosticArray diagnostics;
    diagnostics.header.stamp = ros::Time::now();
    diagnostics.status.push_back(stats_->diagnostic());
    diagnostics_publisher_.publish(diagnostics);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher status_publisher_;
  ros::Publisher diagnostics_publisher_;
  std::shared_ptr<StreamStats> stats_;
  std::unique_ptr<UdpWorker> worker_;
  ros::WallTimer diagnostics_timer_;
};

}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "competition_vehicle_status_receiver_node");
  try {
    morai_udp_bridge::CompetitionVehicleStatusReceiverNode receiver;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start Competition Vehicle Status receiver: %s",
              error.what());
    return 1;
  } catch (...) {
    ROS_FATAL(
        "failed to start Competition Vehicle Status receiver: unknown exception");
    return 1;
  }
  return 0;
}
