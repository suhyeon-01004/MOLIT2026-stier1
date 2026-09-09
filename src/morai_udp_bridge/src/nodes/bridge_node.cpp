#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>

#include "morai_udp_bridge/sensors/protocol.hpp"
#include "morai_udp_bridge/sensors/streams.hpp"
#include "morai_udp_bridge/network/transport.hpp"

namespace morai_udp_bridge {
namespace {

using XmlValue = XmlRpc::XmlRpcValue;

void requireStruct(const XmlValue& value, const std::string& path) {
  if (value.getType() != XmlValue::TypeStruct) {
    throw std::invalid_argument(path + " must be a YAML mapping");
  }
}

const XmlValue& requiredMember(const XmlValue& mapping, const std::string& key,
                               const std::string& path) {
  requireStruct(mapping, path);
  if (!mapping.hasMember(key)) {
    throw std::invalid_argument(path + "/" + key + " is missing");
  }
  return mapping[key];
}

std::string asString(const XmlValue& value, const std::string& path) {
  if (value.getType() != XmlValue::TypeString) {
    throw std::invalid_argument(path + " must be a string");
  }
  return static_cast<std::string>(value);
}

double asDouble(const XmlValue& value, const std::string& path) {
  if (value.getType() == XmlValue::TypeDouble) {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlValue::TypeInt) {
    return static_cast<int>(value);
  }
  throw std::invalid_argument(path + " must be numeric");
}

int asInt(const XmlValue& value, const std::string& path) {
  if (value.getType() != XmlValue::TypeInt) {
    throw std::invalid_argument(path + " must be an integer");
  }
  return static_cast<int>(value);
}

bool asBool(const XmlValue& value, const std::string& path) {
  if (value.getType() != XmlValue::TypeBoolean) {
    throw std::invalid_argument(path + " must be true or false");
  }
  return static_cast<bool>(value);
}

std::string optionalString(const XmlValue& mapping, const std::string& key,
                           const std::string& fallback, const std::string& path) {
  return mapping.hasMember(key) ? asString(mapping[key], path + "/" + key) : fallback;
}

double optionalDouble(const XmlValue& mapping, const std::string& key, double fallback,
                      const std::string& path) {
  return mapping.hasMember(key) ? asDouble(mapping[key], path + "/" + key) : fallback;
}

int optionalInt(const XmlValue& mapping, const std::string& key, int fallback,
                const std::string& path) {
  return mapping.hasMember(key) ? asInt(mapping[key], path + "/" + key) : fallback;
}

bool optionalBool(const XmlValue& mapping, const std::string& key, bool fallback,
                  const std::string& path) {
  return mapping.hasMember(key) ? asBool(mapping[key], path + "/" + key) : fallback;
}

std::uint16_t portFromConfig(const XmlValue& config, const std::string& path) {
  const int port = asInt(requiredMember(config, "port", path), path + "/port");
  if (port < 1 || port > 65535) {
    throw std::invalid_argument(path + "/port is outside 1..65535");
  }
  return static_cast<std::uint16_t>(port);
}

std::size_t bufferFromConfig(const XmlValue& config, std::size_t fallback,
                             const std::string& path) {
  if (!config.hasMember("receive_buffer_bytes")) {
    return fallback;
  }
  const int configured = asInt(config["receive_buffer_bytes"],
                               path + "/receive_buffer_bytes");
  if (configured <= 0) {
    throw std::invalid_argument(path + "/receive_buffer_bytes must be positive");
  }
  return static_cast<std::size_t>(configured);
}

}  // namespace

class MoraiUdpBridgeCpp {
 public:
  MoraiUdpBridgeCpp() : node_(), private_node_("~") {
    private_node_.param<std::string>("bind_ip", bind_ip_, "0.0.0.0");
    private_node_.param<std::string>("allowed_source_ip", allowed_source_ip_, "");

    int receive_buffer = 4 * 1024 * 1024;
    private_node_.param("receive_buffer_bytes", receive_buffer, receive_buffer);
    if (receive_buffer <= 0) {
      throw std::invalid_argument("receive_buffer_bytes must be positive");
    }
    receive_buffer_ = static_cast<std::size_t>(receive_buffer);

    double diagnostics_period = 1.0;
    private_node_.param("diagnostics_period", diagnostics_period, diagnostics_period);
    if (diagnostics_period <= 0.0) {
      throw std::invalid_argument("diagnostics_period must be positive");
    }

    XmlValue sensor_setup;
    if (!private_node_.getParam("sensor_setup", sensor_setup)) {
      throw std::invalid_argument("sensor_setup parameter is missing");
    }
    requireStruct(sensor_setup, "sensor_setup");

    XmlValue cameras;
    if (private_node_.getParam("cameras", cameras)) {
      configureCameras(cameras, sensor_setup);
    }

    XmlValue gps;
    if (private_node_.getParam("gps", gps)) {
      configureGps(gps, sensor_setup);
    }

    XmlValue imu;
    if (private_node_.getParam("imu", imu)) {
      configureImu(imu, sensor_setup);
    }

    if (workers_.empty()) {
      throw std::runtime_error("no UDP sensor streams are enabled");
    }

    diagnostics_publisher_ =
        node_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    for (const auto& worker : workers_) {
      worker->start();
    }
    diagnostics_timer_ = node_.createTimer(ros::Duration(diagnostics_period),
                                           &MoraiUdpBridgeCpp::publishDiagnostics, this);
  }

  ~MoraiUdpBridgeCpp() {
    diagnostics_timer_.stop();
    for (const auto& worker : workers_) {
      worker->close();
    }
  }

  MoraiUdpBridgeCpp(const MoraiUdpBridgeCpp&) = delete;
  MoraiUdpBridgeCpp& operator=(const MoraiUdpBridgeCpp&) = delete;

 private:
  struct CommonConfig {
    std::string path;
    std::string bind_ip;
    std::string allowed_source_ip;
    std::string topic;
    std::uint16_t port{0U};
    std::size_t receive_buffer{0U};
    double stale_timeout{1.0};
    double max_hz{0.0};
  };

  CommonConfig commonConfig(const XmlValue& config, const std::string& path) const {
    requireStruct(config, path);
    CommonConfig output;
    output.path = path;
    output.bind_ip = optionalString(config, "bind_ip", bind_ip_, path);
    output.allowed_source_ip =
        optionalString(config, "allowed_source_ip", allowed_source_ip_, path);
    output.topic = asString(requiredMember(config, "topic", path), path + "/topic");
    output.port = portFromConfig(config, path);
    output.receive_buffer = bufferFromConfig(config, receive_buffer_, path);
    output.stale_timeout = optionalDouble(config, "stale_timeout", 1.0, path);
    output.max_hz = optionalDouble(config, "max_hz", 0.0, path);
    if (output.stale_timeout <= 0.0) {
      throw std::invalid_argument(path + "/stale_timeout must be positive");
    }
    if (output.max_hz < 0.0) {
      throw std::invalid_argument(path + "/max_hz cannot be negative");
    }
    return output;
  }

  std::shared_ptr<StreamStats> addStats(const std::string& name,
                                        const CommonConfig& config) {
    auto stats = std::make_shared<StreamStats>(name, config.bind_ip, config.port,
                                               config.topic, config.stale_timeout,
                                               config.max_hz);
    stats_.push_back(stats);
    return stats;
  }

  template <typename Callback>
  void addWorker(const CommonConfig& config, std::shared_ptr<StreamStats> stats,
                 Callback callback) {
    const auto endpoint = std::make_pair(config.bind_ip, config.port);
    if (!bound_endpoints_.insert(endpoint).second) {
      throw std::runtime_error("duplicate UDP bind endpoint " + config.bind_ip + ":" +
                               std::to_string(config.port));
    }
    workers_.emplace_back(new UdpWorker(config.bind_ip, config.port, callback,
                                        std::move(stats), config.receive_buffer,
                                        config.allowed_source_ip));
  }

  void configureCameras(const XmlValue& cameras, const XmlValue& sensor_setup) {
    requireStruct(cameras, "cameras");
    const XmlValue& camera_setup = requiredMember(sensor_setup, "cameras", "sensor_setup");
    requireStruct(camera_setup, "sensor_setup/cameras");

    for (auto iterator = cameras.begin(); iterator != cameras.end(); ++iterator) {
      const std::string name = iterator->first;
      const XmlValue& transport = iterator->second;
      const std::string path = "cameras/" + name;
      if (!camera_setup.hasMember(name)) {
        throw std::invalid_argument("sensor_setup/cameras/" + name + " is missing");
      }
      const XmlValue& model = camera_setup[name];
      requireStruct(model, "sensor_setup/cameras/" + name);
      const CommonConfig common = commonConfig(transport, path);

      CameraConfig config;
      config.name = name;
      config.topic = common.topic;
      config.camera_info_topic =
          asString(requiredMember(transport, "camera_info_topic", path),
                   path + "/camera_info_topic");
      config.frame_id =
          asString(requiredMember(model, "optical_frame_id", "sensor_setup/cameras/" + name),
                   "sensor_setup/cameras/" + name + "/optical_frame_id");
      config.width = static_cast<std::uint32_t>(asInt(
          requiredMember(model, "width", "sensor_setup/cameras/" + name),
          "sensor_setup/cameras/" + name + "/width"));
      config.height = static_cast<std::uint32_t>(asInt(
          requiredMember(model, "height", "sensor_setup/cameras/" + name),
          "sensor_setup/cameras/" + name + "/height"));
      config.horizontal_fov_deg = asDouble(
          requiredMember(model, "horizontal_fov_deg", "sensor_setup/cameras/" + name),
          "sensor_setup/cameras/" + name + "/horizontal_fov_deg");
      config.packet_layout = packetLayoutFromString(
          optionalString(transport, "packet_layout", "auto", path));
      config.use_sensor_time = optionalBool(transport, "use_sensor_time", false, path);
      const int max_chunks = optionalInt(transport, "max_chunks", 256, path);
      if (max_chunks <= 0) {
        throw std::invalid_argument(path + "/max_chunks must be positive");
      }
      config.max_chunks = static_cast<std::size_t>(max_chunks);
      if (transport.hasMember("cx")) {
        config.has_cx = true;
        config.cx = asDouble(transport["cx"], path + "/cx");
      }
      if (transport.hasMember("cy")) {
        config.has_cy = true;
        config.cy = asDouble(transport["cy"], path + "/cy");
      }

      const auto stats = addStats("camera_" + name, common);
      const auto stream = std::make_shared<CameraStream>(node_, config, stats);
      addWorker(common, stats, [stream](const std::uint8_t* data, std::size_t size) {
        stream->handle(data, size);
      });
    }
  }

  void configureGps(const XmlValue& transport, const XmlValue& sensor_setup) {
    const std::string path = "gps";
    requireStruct(transport, path);
    if (!optionalBool(transport, "enabled", true, path)) {
      return;
    }
    const XmlValue& model = requiredMember(sensor_setup, "gps", "sensor_setup");
    const CommonConfig common = commonConfig(transport, path);
    GpsConfig config;
    config.topic = common.topic;
    config.frame_id =
        asString(requiredMember(model, "frame_id", "sensor_setup/gps"),
                 "sensor_setup/gps/frame_id");
    const auto stats = addStats("gps", common);
    const auto stream = std::make_shared<GpsStream>(node_, config, stats);
    addWorker(common, stats, [stream](const std::uint8_t* data, std::size_t size) {
      stream->handle(data, size);
    });
  }

  void configureImu(const XmlValue& transport, const XmlValue& sensor_setup) {
    const std::string path = "imu";
    requireStruct(transport, path);
    if (!optionalBool(transport, "enabled", true, path)) {
      return;
    }
    const XmlValue& model = requiredMember(sensor_setup, "imu", "sensor_setup");
    const CommonConfig common = commonConfig(transport, path);
    ImuConfig config;
    config.topic = common.topic;
    config.frame_id =
        asString(requiredMember(model, "frame_id", "sensor_setup/imu"),
                 "sensor_setup/imu/frame_id");
    config.packet_layout = packetLayoutFromString(
        optionalString(transport, "packet_layout", "auto", path));
    config.use_sensor_time = optionalBool(transport, "use_sensor_time", false, path);
    const auto stats = addStats("imu", common);
    const auto stream = std::make_shared<ImuStream>(node_, config, stats);
    addWorker(common, stats, [stream](const std::uint8_t* data, std::size_t size) {
      stream->handle(data, size);
    });
  }

  void publishDiagnostics(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray message;
    message.header.stamp = ros::Time::now();
    message.status.reserve(stats_.size());
    for (const auto& stats : stats_) {
      message.status.push_back(stats->diagnostic());
    }
    diagnostics_publisher_.publish(message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string bind_ip_;
  std::string allowed_source_ip_;
  std::size_t receive_buffer_{0U};
  std::set<std::pair<std::string, std::uint16_t>> bound_endpoints_;
  std::vector<std::shared_ptr<StreamStats>> stats_;
  std::vector<std::unique_ptr<UdpWorker>> workers_;
  ros::Publisher diagnostics_publisher_;
  ros::Timer diagnostics_timer_;
};

}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ros::init(argc, argv, "morai_udp_bridge");
  try {
    morai_udp_bridge::MoraiUdpBridgeCpp bridge;
    ROS_INFO("MORAI C++ UDP sensor bridge is listening for camera, GPS, and IMU data");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("failed to start MORAI C++ UDP bridge: %s", error.what());
    return 1;
  } catch (...) {
    ROS_FATAL("failed to start MORAI C++ UDP bridge: unknown exception");
    return 1;
  }
  return 0;
}
