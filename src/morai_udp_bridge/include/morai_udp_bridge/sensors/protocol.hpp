#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace morai_udp_bridge {

class ProtocolError : public std::runtime_error {
 public:
  explicit ProtocolError(const std::string& message) : std::runtime_error(message) {}
};

// A GPS stream may deliver other NMEA sentence types (for example RMC) in
// datagrams separate from GGA. This condition is not packet damage.
class GgaSentenceNotFound : public ProtocolError {
 public:
  GgaSentenceNotFound()
      : ProtocolError("GPS datagram does not contain a GGA sentence") {}
};

enum class PacketLayout { kAuto, kLegacy, kTimestamped };

PacketLayout packetLayoutFromString(const std::string& value);
const char* packetLayoutName(PacketLayout layout);

struct CameraPacket {
  PacketLayout layout{PacketLayout::kLegacy};
  std::uint32_t index{0};
  std::uint32_t declared_size{0};
  std::vector<std::uint8_t> payload;
  bool is_final{false};
  bool has_timestamp{false};
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
};

struct AssembledFrame {
  std::vector<std::uint8_t> jpeg;
  bool has_timestamp{false};
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
  std::size_t packet_count{0};
};

CameraPacket parseCameraPacket(const std::uint8_t* data, std::size_t size,
                               PacketLayout layout = PacketLayout::kAuto);

inline CameraPacket parseCameraPacket(const std::vector<std::uint8_t>& raw,
                                      PacketLayout layout = PacketLayout::kAuto) {
  return parseCameraPacket(raw.data(), raw.size(), layout);
}

class CameraFrameAssembler {
 public:
  explicit CameraFrameAssembler(std::size_t max_chunks = 256);

  // Returns true and replaces `frame` when a complete JPEG is available.
  bool add(CameraPacket packet, AssembledFrame* frame);

  std::uint64_t droppedFrames() const { return dropped_frames_; }
  std::uint64_t invalidFrames() const { return invalid_frames_; }

 private:
  void reset();
  void dropCurrent(bool invalid = false);

  std::size_t max_chunks_;
  std::uint64_t dropped_frames_{0};
  std::uint64_t invalid_frames_{0};
  std::map<std::uint32_t, std::vector<std::uint8_t>> chunks_;
  bool has_key_{false};
  std::uint32_t key_sec_{0};
  std::uint32_t key_nsec_{0};
  bool has_start_index_{false};
  std::uint32_t start_index_{0};
  bool has_final_index_{false};
  std::uint32_t final_index_{0};
};

struct ImuSample {
  double orientation_w{0.0};
  double orientation_x{0.0};
  double orientation_y{0.0};
  double orientation_z{0.0};
  double angular_velocity_x{0.0};
  double angular_velocity_y{0.0};
  double angular_velocity_z{0.0};
  double linear_acceleration_x{0.0};
  double linear_acceleration_y{0.0};
  double linear_acceleration_z{0.0};
  bool has_timestamp{false};
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
  PacketLayout layout{PacketLayout::kLegacy};
};

ImuSample parseImuPacket(const std::uint8_t* data, std::size_t size,
                         PacketLayout layout = PacketLayout::kAuto);

inline ImuSample parseImuPacket(const std::vector<std::uint8_t>& raw,
                                PacketLayout layout = PacketLayout::kAuto) {
  return parseImuPacket(raw.data(), raw.size(), layout);
}

struct GgaFix {
  std::string utc;
  bool has_latitude{false};
  bool has_longitude{false};
  double latitude{0.0};
  double longitude{0.0};
  int quality{0};
  int satellites{0};
  bool has_hdop{false};
  double hdop{0.0};
  bool has_altitude_msl{false};
  double altitude_msl{0.0};
  bool has_geoid_separation{false};
  double geoid_separation{0.0};

  bool valid() const {
    return quality > 0 && has_latitude && has_longitude;
  }
  bool hasAltitudeEllipsoid() const { return has_altitude_msl; }
  double altitudeEllipsoid() const {
    return altitude_msl + (has_geoid_separation ? geoid_separation : 0.0);
  }
};

GgaFix parseGga(const std::string& sentence);
GgaFix parseGgaDatagram(const std::uint8_t* data, std::size_t size);

inline GgaFix parseGgaDatagram(const std::vector<std::uint8_t>& raw) {
  return parseGgaDatagram(raw.data(), raw.size());
}

}  // namespace morai_udp_bridge
