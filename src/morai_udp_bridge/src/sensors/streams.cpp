#include "morai_udp_bridge/sensors/streams.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

namespace morai_udp_bridge {

ros::Time rosStamp(bool has_timestamp, std::uint32_t sec, std::uint32_t nsec,
                   bool use_sensor_time) {
  if (use_sensor_time && has_timestamp && (sec != 0U || nsec != 0U)) {
    return ros::Time(sec, nsec);
  }
  return ros::Time::now();
}

sensor_msgs::CameraInfo cameraInfoFromConfig(const CameraConfig& config) {
  if (config.width == 0U || config.height == 0U) {
    throw std::invalid_argument("camera width and height must be positive");
  }
  if (!(config.horizontal_fov_deg > 0.0 && config.horizontal_fov_deg < 180.0)) {
    throw std::invalid_argument("camera horizontal_fov_deg must be between 0 and 180");
  }
  constexpr double kPi = 3.14159265358979323846;
  const double horizontal_fov = config.horizontal_fov_deg * kPi / 180.0;
  const double fx = static_cast<double>(config.width) / (2.0 * std::tan(horizontal_fov / 2.0));
  const double fy = fx;
  const double cx = config.has_cx ? config.cx : static_cast<double>(config.width) / 2.0;
  const double cy = config.has_cy ? config.cy : static_cast<double>(config.height) / 2.0;

  sensor_msgs::CameraInfo message;
  message.width = config.width;
  message.height = config.height;
  message.distortion_model = "plumb_bob";
  message.D.assign(5U, 0.0);
  message.K = {{fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}};
  message.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0}};
  message.P = {{fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0}};
  return message;
}

CameraStream::CameraStream(ros::NodeHandle& node, const CameraConfig& config,
                           std::shared_ptr<StreamStats> stats)
    : config_(config),
      stats_(std::move(stats)),
      assembler_(config.max_chunks),
      image_publisher_(node.advertise<sensor_msgs::CompressedImage>(config.topic, 1)),
      info_publisher_(node.advertise<sensor_msgs::CameraInfo>(config.camera_info_topic, 1)),
      info_template_(cameraInfoFromConfig(config)) {
  if (!stats_) {
    throw std::invalid_argument("camera stats object is null");
  }
}

void CameraStream::handle(const std::uint8_t* data, std::size_t size) {
  CameraPacket packet;
  try {
    packet = parseCameraPacket(data, size, config_.packet_layout);
  } catch (const ProtocolError&) {
    stats_->parseError();
    return;
  }

  AssembledFrame frame;
  const bool complete = assembler_.add(std::move(packet), &frame);
  stats_->setDropped(assembler_.droppedFrames());
  if (!complete) {
    return;
  }

  sensor_msgs::CompressedImage image;
  image.header.stamp =
      rosStamp(frame.has_timestamp, frame.sec, frame.nsec, config_.use_sensor_time);
  image.header.frame_id = config_.frame_id;
  image.format = "jpeg";
  image.data = std::move(frame.jpeg);
  image_publisher_.publish(image);

  sensor_msgs::CameraInfo info = info_template_;
  info.header = image.header;
  info_publisher_.publish(info);
  stats_->published();
}

GpsStream::GpsStream(ros::NodeHandle& node, const GpsConfig& config,
                     std::shared_ptr<StreamStats> stats)
    : config_(config),
      stats_(std::move(stats)),
      publisher_(node.advertise<sensor_msgs::NavSatFix>(config.topic, 10)) {
  if (!stats_) {
    throw std::invalid_argument("GPS stats object is null");
  }
}

void GpsStream::handle(const std::uint8_t* data, std::size_t size) {
  GgaFix sample;
  try {
    sample = parseGgaDatagram(data, size);
  } catch (const GgaSentenceNotFound&) {
    // MORAI sends non-GGA NMEA sentences as separate UDP datagrams. They do
    // not contain the position/altitude fields used by NavSatFix.
    return;
  } catch (const ProtocolError& error) {
    stats_->parseError();
    ROS_WARN_STREAM_THROTTLE(
        5.0, "failed to parse MORAI GPS packet (" << size
                                                   << " bytes): "
                                                   << error.what());
    return;
  }

  sensor_msgs::NavSatFix message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = config_.frame_id;
  message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  if (!sample.valid()) {
    message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    message.latitude = std::numeric_limits<double>::quiet_NaN();
    message.longitude = std::numeric_limits<double>::quiet_NaN();
    message.altitude = std::numeric_limits<double>::quiet_NaN();
  } else {
    message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    message.latitude = sample.latitude;
    message.longitude = sample.longitude;
    message.altitude = sample.hasAltitudeEllipsoid()
                           ? sample.altitudeEllipsoid()
                           : std::numeric_limits<double>::quiet_NaN();
  }
  publisher_.publish(message);
  stats_->published();
}

ImuStream::ImuStream(ros::NodeHandle& node, const ImuConfig& config,
                     std::shared_ptr<StreamStats> stats)
    : config_(config),
      stats_(std::move(stats)),
      publisher_(node.advertise<sensor_msgs::Imu>(config.topic, 50)) {
  if (!stats_) {
    throw std::invalid_argument("IMU stats object is null");
  }
}

void ImuStream::handle(const std::uint8_t* data, std::size_t size) {
  ImuSample sample;
  try {
    sample = parseImuPacket(data, size, config_.packet_layout);
  } catch (const ProtocolError& error) {
    stats_->parseError();
    ROS_WARN_STREAM_THROTTLE(
        5.0, "failed to parse MORAI IMU packet (" << size
                                                   << " bytes): "
                                                   << error.what());
    return;
  }

  sensor_msgs::Imu message;
  message.header.stamp =
      rosStamp(sample.has_timestamp, sample.sec, sample.nsec, config_.use_sensor_time);
  message.header.frame_id = config_.frame_id;
  message.orientation.w = sample.orientation_w;
  message.orientation.x = sample.orientation_x;
  message.orientation.y = sample.orientation_y;
  message.orientation.z = sample.orientation_z;
  message.angular_velocity.x = sample.angular_velocity_x;
  message.angular_velocity.y = sample.angular_velocity_y;
  message.angular_velocity.z = sample.angular_velocity_z;
  message.linear_acceleration.x = sample.linear_acceleration_x;
  message.linear_acceleration.y = sample.linear_acceleration_y;
  message.linear_acceleration.z = sample.linear_acceleration_z;
  // MORAI transmits no covariance. The zero-initialized matrices preserve the
  // existing bridge message contract.
  publisher_.publish(message);
  stats_->published();
}

}  // namespace morai_udp_bridge
