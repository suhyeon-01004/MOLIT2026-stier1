#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include "morai_udp_bridge/sensors/protocol.hpp"
#include "morai_udp_bridge/network/transport.hpp"

namespace morai_udp_bridge {

struct CameraConfig {
  std::string name;
  std::string topic;
  std::string camera_info_topic;
  std::string frame_id;
  PacketLayout packet_layout{PacketLayout::kAuto};
  bool use_sensor_time{false};
  std::size_t max_chunks{256};
  std::uint32_t width{0};
  std::uint32_t height{0};
  double horizontal_fov_deg{0.0};
  bool has_cx{false};
  bool has_cy{false};
  double cx{0.0};
  double cy{0.0};
};

struct GpsConfig {
  std::string topic;
  std::string frame_id;
};

struct ImuConfig {
  std::string topic;
  std::string frame_id;
  PacketLayout packet_layout{PacketLayout::kAuto};
  bool use_sensor_time{false};
};

ros::Time rosStamp(bool has_timestamp, std::uint32_t sec, std::uint32_t nsec,
                   bool use_sensor_time);
sensor_msgs::CameraInfo cameraInfoFromConfig(const CameraConfig& config);

class CameraStream {
 public:
  CameraStream(ros::NodeHandle& node, const CameraConfig& config,
               std::shared_ptr<StreamStats> stats);
  void handle(const std::uint8_t* data, std::size_t size);

 private:
  const CameraConfig config_;
  const std::shared_ptr<StreamStats> stats_;
  CameraFrameAssembler assembler_;
  ros::Publisher image_publisher_;
  ros::Publisher info_publisher_;
  sensor_msgs::CameraInfo info_template_;
};

class GpsStream {
 public:
  GpsStream(ros::NodeHandle& node, const GpsConfig& config,
            std::shared_ptr<StreamStats> stats);
  void handle(const std::uint8_t* data, std::size_t size);

 private:
  const GpsConfig config_;
  const std::shared_ptr<StreamStats> stats_;
  ros::Publisher publisher_;
};

class ImuStream {
 public:
  ImuStream(ros::NodeHandle& node, const ImuConfig& config,
            std::shared_ptr<StreamStats> stats);
  void handle(const std::uint8_t* data, std::size_t size);

 private:
  const ImuConfig config_;
  const std::shared_ptr<StreamStats> stats_;
  ros::Publisher publisher_;
};

}  // namespace morai_udp_bridge
