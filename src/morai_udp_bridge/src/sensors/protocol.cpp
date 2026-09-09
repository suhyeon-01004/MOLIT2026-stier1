#include "morai_udp_bridge/sensors/protocol.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace morai_udp_bridge {
namespace {

constexpr std::uint32_t kMaximumChunkIndex = 10000U;
constexpr std::uint32_t kMaximumDeclaredJpegSize = 100U * 1024U * 1024U;
constexpr std::uint32_t kNanosecondsPerSecond = 1000000000U;

std::uint32_t readLe32(const std::uint8_t* data) {
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

std::uint64_t readLe64(const std::uint8_t* data) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < 8; ++index) {
    value |= static_cast<std::uint64_t>(data[index]) << (8U * index);
  }
  return value;
}

double readLeDouble(const std::uint8_t* data) {
  const std::uint64_t bits = readLe64(data);
  double value = 0.0;
  static_assert(sizeof(value) == sizeof(bits), "double must be IEEE-754 binary64");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

bool timestampedCameraHeaderIsPlausible(const std::uint8_t* data, std::size_t size) {
  if (size < 21U) {
    return false;
  }
  const std::uint32_t nsec = readLe32(data + 7U);
  const std::uint32_t index = readLe32(data + 11U);
  const std::uint32_t declared_size = readLe32(data + 15U);
  return nsec < kNanosecondsPerSecond && index < kMaximumChunkIndex &&
         declared_size < kMaximumDeclaredJpegSize;
}

bool startsWithJpegSoi(const std::vector<std::uint8_t>& payload) {
  return payload.size() >= 2U && payload[0] == 0xffU && payload[1] == 0xd8U;
}

std::string trimNmea(const std::string& value) {
  const std::string whitespace("\0\r\n ", 4U);
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return std::string();
  }
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1U);
}

std::vector<std::string> splitPreservingEmpty(const std::string& value, char delimiter) {
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (true) {
    const std::size_t end = value.find(delimiter, begin);
    if (end == std::string::npos) {
      fields.emplace_back(value.substr(begin));
      return fields;
    }
    fields.emplace_back(value.substr(begin, end - begin));
    begin = end + 1U;
  }
}

long parseLongStrict(const std::string& value, const char* field_name) {
  if (value.empty()) {
    throw ProtocolError(std::string("GGA ") + field_name + " is empty");
  }
  errno = 0;
  char* end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (errno == ERANGE || end == value.c_str() || *end != '\0') {
    throw ProtocolError(std::string("GGA ") + field_name + " is malformed");
  }
  return parsed;
}

double parseDoubleStrict(const std::string& value, const char* field_name) {
  if (value.empty()) {
    throw ProtocolError(std::string("GGA ") + field_name + " is empty");
  }
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (errno == ERANGE || end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    throw ProtocolError(std::string("GGA ") + field_name + " is malformed");
  }
  return parsed;
}

bool optionalDouble(const std::string& value, const char* field_name, double* output) {
  if (value.empty()) {
    return false;
  }
  *output = parseDoubleStrict(value, field_name);
  return true;
}

std::string checksumBody(const std::string& sentence) {
  if (sentence.empty() || sentence.front() != '$') {
    throw ProtocolError("NMEA sentence does not start with $");
  }
  const std::string stripped = trimNmea(sentence);
  const std::size_t separator = stripped.rfind('*');
  if (separator == std::string::npos) {
    throw ProtocolError("NMEA checksum is missing");
  }
  const std::string body = stripped.substr(1U, separator - 1U);
  const std::string checksum_text = stripped.substr(separator + 1U);
  if (checksum_text.size() != 2U) {
    throw ProtocolError("NMEA checksum must contain exactly two hexadecimal digits");
  }

  unsigned int expected = 0U;
  std::istringstream input(checksum_text);
  input >> std::hex >> expected;
  if (!input || !input.eof() || expected > 0xffU) {
    throw ProtocolError("NMEA checksum is not hexadecimal");
  }

  std::uint8_t checksum = 0U;
  for (const unsigned char character : body) {
    checksum ^= character;
  }
  if (checksum != static_cast<std::uint8_t>(expected)) {
    throw ProtocolError("NMEA checksum mismatch");
  }
  return body;
}

double coordinate(const std::string& value, const std::string& direction,
                  bool is_latitude) {
  const bool direction_valid =
      is_latitude ? (direction == "N" || direction == "S")
                  : (direction == "E" || direction == "W");
  if (value.empty() || !direction_valid) {
    throw ProtocolError("NMEA coordinate is incomplete");
  }
  const double raw = parseDoubleStrict(value, "coordinate");
  if (raw < 0.0) {
    throw ProtocolError("NMEA coordinate is out of range");
  }
  const int degrees = static_cast<int>(std::floor(raw / 100.0));
  const double minutes = raw - static_cast<double>(degrees) * 100.0;
  double result = static_cast<double>(degrees) + minutes / 60.0;
  const double limit = is_latitude ? 90.0 : 180.0;
  if (minutes >= 60.0 || result > limit) {
    throw ProtocolError("NMEA coordinate is out of range");
  }
  if (direction == "S" || direction == "W") {
    result = -result;
  }
  return result;
}

}  // namespace

PacketLayout packetLayoutFromString(const std::string& value) {
  if (value == "auto") {
    return PacketLayout::kAuto;
  }
  if (value == "legacy") {
    return PacketLayout::kLegacy;
  }
  if (value == "timestamped") {
    return PacketLayout::kTimestamped;
  }
  throw ProtocolError("unknown packet layout: " + value);
}

const char* packetLayoutName(PacketLayout layout) {
  switch (layout) {
    case PacketLayout::kAuto:
      return "auto";
    case PacketLayout::kLegacy:
      return "legacy";
    case PacketLayout::kTimestamped:
      return "timestamped";
  }
  return "unknown";
}

CameraPacket parseCameraPacket(const std::uint8_t* data, std::size_t size,
                               PacketLayout layout) {
  if (data == nullptr) {
    throw ProtocolError("camera packet data is null");
  }
  if (size < 13U) {
    throw ProtocolError("camera packet is shorter than 13 bytes");
  }
  if (data[0] != 'M' || data[1] != 'O' || data[2] != 'R') {
    throw ProtocolError("camera packet header is not MOR");
  }

  PacketLayout selected = layout;
  if (selected == PacketLayout::kAuto) {
    selected = timestampedCameraHeaderIsPlausible(data, size)
                   ? PacketLayout::kTimestamped
                   : PacketLayout::kLegacy;
  }

  CameraPacket packet;
  packet.layout = selected;
  std::size_t payload_offset = 0U;
  if (selected == PacketLayout::kTimestamped) {
    if (size < 21U) {
      throw ProtocolError("timestamped camera packet is too short");
    }
    packet.has_timestamp = true;
    packet.sec = readLe32(data + 3U);
    packet.nsec = readLe32(data + 7U);
    packet.index = readLe32(data + 11U);
    packet.declared_size = readLe32(data + 15U);
    payload_offset = 19U;
    if (packet.nsec >= kNanosecondsPerSecond) {
      throw ProtocolError("camera nsec is out of range");
    }
  } else if (selected == PacketLayout::kLegacy) {
    packet.index = readLe32(data + 3U);
    packet.declared_size = readLe32(data + 7U);
    payload_offset = 11U;
  } else {
    throw ProtocolError("camera parser received unresolved packet layout");
  }

  if (packet.index >= kMaximumChunkIndex) {
    throw ProtocolError("camera chunk index is implausibly large");
  }
  if (packet.declared_size == 0U ||
      packet.declared_size >= kMaximumDeclaredJpegSize) {
    throw ProtocolError("camera declared size is invalid");
  }

  const std::uint8_t tail_first = data[size - 2U];
  const std::uint8_t tail_second = data[size - 1U];
  if (!((tail_first == 'A' || tail_first == 'E') && tail_second == 'I')) {
    throw ProtocolError("camera packet tail is neither AI nor EI");
  }
  packet.is_final = tail_first == 'E';

  const std::size_t payload_size = size - payload_offset - 2U;
  std::size_t retained_size = payload_size;
  // MORAI versions disagree whether Size denotes a chunk or the whole JPEG.
  // Crop only when Size fits in this datagram.
  if (packet.declared_size <= payload_size) {
    retained_size = packet.declared_size;
  }
  packet.payload.assign(data + payload_offset, data + payload_offset + retained_size);
  return packet;
}

CameraFrameAssembler::CameraFrameAssembler(std::size_t max_chunks)
    : max_chunks_(max_chunks) {
  if (max_chunks_ == 0U) {
    throw std::invalid_argument("camera max_chunks must be positive");
  }
}

void CameraFrameAssembler::reset() {
  chunks_.clear();
  has_key_ = false;
  key_sec_ = 0U;
  key_nsec_ = 0U;
  has_start_index_ = false;
  start_index_ = 0U;
  has_final_index_ = false;
  final_index_ = 0U;
}

void CameraFrameAssembler::dropCurrent(bool invalid) {
  if (!chunks_.empty()) {
    ++dropped_frames_;
    if (invalid) {
      ++invalid_frames_;
    }
  }
  reset();
}

bool CameraFrameAssembler::add(CameraPacket packet, AssembledFrame* frame) {
  if (frame == nullptr) {
    throw std::invalid_argument("assembled frame output is null");
  }

  if (packet.has_timestamp) {
    if (has_key_ && (packet.sec != key_sec_ || packet.nsec != key_nsec_)) {
      dropCurrent();
    }
    has_key_ = true;
    key_sec_ = packet.sec;
    key_nsec_ = packet.nsec;
  } else if ((packet.index == 0U || packet.index == 1U) && !chunks_.empty() &&
             chunks_.count(packet.index) != 0U) {
    dropCurrent();
  }

  if (packet.index == 0U) {
    has_start_index_ = true;
    start_index_ = 0U;
  } else if (packet.index == 1U && !has_start_index_ &&
             startsWithJpegSoi(packet.payload)) {
    has_start_index_ = true;
    start_index_ = 1U;
  }

  chunks_[packet.index] = std::move(packet.payload);
  if (chunks_.size() > max_chunks_) {
    dropCurrent(true);
    return false;
  }

  if (packet.is_final) {
    has_final_index_ = true;
    final_index_ = packet.index;
  }
  if (!has_start_index_ || !has_final_index_) {
    return false;
  }
  if (final_index_ < start_index_) {
    dropCurrent(true);
    return false;
  }

  std::size_t total_size = 0U;
  for (std::uint32_t index = start_index_;; ++index) {
    const auto chunk = chunks_.find(index);
    if (chunk == chunks_.end()) {
      return false;
    }
    total_size += chunk->second.size();
    if (index == final_index_) {
      break;
    }
  }

  std::vector<std::uint8_t> data;
  data.reserve(total_size);
  for (std::uint32_t index = start_index_;; ++index) {
    const auto& chunk = chunks_.at(index);
    data.insert(data.end(), chunk.begin(), chunk.end());
    if (index == final_index_) {
      break;
    }
  }

  if (!startsWithJpegSoi(data)) {
    dropCurrent(true);
    return false;
  }
  const std::array<std::uint8_t, 2> eoi{{0xffU, 0xd9U}};
  const auto eoi_position = std::search(data.begin() + 2, data.end(), eoi.begin(), eoi.end());
  if (eoi_position == data.end()) {
    dropCurrent(true);
    return false;
  }

  frame->jpeg.assign(data.begin(), eoi_position + 2);
  frame->has_timestamp = has_key_;
  frame->sec = key_sec_;
  frame->nsec = key_nsec_;
  frame->packet_count = static_cast<std::size_t>(final_index_ - start_index_) + 1U;
  reset();
  return true;
}

ImuSample parseImuPacket(const std::uint8_t* data, std::size_t size,
                         PacketLayout layout) {
  if (data == nullptr) {
    throw ProtocolError("IMU packet data is null");
  }
  if (size != 107U && size != 115U) {
    throw ProtocolError("IMU packet length is neither 107 nor 115 bytes");
  }
  static const std::array<std::uint8_t, 9> kHeader{{'#', 'I', 'M', 'U', 'D', 'a', 't', 'a', '$'}};
  if (!std::equal(kHeader.begin(), kHeader.end(), data)) {
    throw ProtocolError("IMU packet header is not #IMUData$");
  }
  // MORAI versions disagree on whether this field describes only the
  // 80-byte measurement block or also version-specific metadata. The official
  // SensorExample reads but does not validate it. Packet length, tail, fixed
  // measurement offsets and finite values are validated below.
  // Current MORAI releases may populate this 12-byte auxiliary header even
  // though older protocol documentation describes it as zero-filled. The
  // official MORAI SensorExample skips it and parses from the fixed offsets
  // below, so auxiliary contents must not invalidate an IMU packet.

  PacketLayout selected = layout;
  if (selected == PacketLayout::kAuto) {
    if (size == 115U && data[113U] == '\r' && data[114U] == '\n') {
      selected = PacketLayout::kTimestamped;
    } else if (size == 107U && data[105U] == '\r' && data[106U] == '\n') {
      selected = PacketLayout::kLegacy;
    } else {
      throw ProtocolError("IMU tail was not found at a supported offset");
    }
  }

  ImuSample sample;
  sample.layout = selected;
  std::size_t data_offset = 0U;
  if (selected == PacketLayout::kTimestamped) {
    if (size != 115U || data[113U] != '\r' || data[114U] != '\n') {
      throw ProtocolError("invalid timestamped IMU packet length or tail");
    }
    sample.has_timestamp = true;
    sample.sec = readLe32(data + 25U);
    sample.nsec = readLe32(data + 29U);
    if (sample.nsec >= kNanosecondsPerSecond) {
      throw ProtocolError("IMU nsec is out of range");
    }
    data_offset = 33U;
  } else if (selected == PacketLayout::kLegacy) {
    if (size != 107U || data[105U] != '\r' || data[106U] != '\n') {
      throw ProtocolError("invalid legacy IMU packet length or tail");
    }
    data_offset = 25U;
  } else {
    throw ProtocolError("IMU parser received unresolved packet layout");
  }

  std::array<double, 10> values{};
  for (std::size_t index = 0U; index < values.size(); ++index) {
    values[index] = readLeDouble(data + data_offset + index * sizeof(double));
    if (!std::isfinite(values[index])) {
      throw ProtocolError("IMU packet contains a non-finite value");
    }
  }
  sample.orientation_w = values[0];
  sample.orientation_x = values[1];
  sample.orientation_y = values[2];
  sample.orientation_z = values[3];
  sample.angular_velocity_x = values[4];
  sample.angular_velocity_y = values[5];
  sample.angular_velocity_z = values[6];
  sample.linear_acceleration_x = values[7];
  sample.linear_acceleration_y = values[8];
  sample.linear_acceleration_z = values[9];
  return sample;
}

GgaFix parseGga(const std::string& sentence) {
  const std::vector<std::string> fields = splitPreservingEmpty(checksumBody(sentence), ',');
  if (fields.empty() || fields[0].size() < 3U ||
      fields[0].compare(fields[0].size() - 3U, 3U, "GGA") != 0) {
    throw ProtocolError("sentence is not GGA");
  }
  if (fields.size() < 15U) {
    throw ProtocolError("GGA sentence has too few fields");
  }

  GgaFix fix;
  fix.utc = fields[1];
  try {
    fix.quality = fields[6].empty() ? 0 : static_cast<int>(parseLongStrict(fields[6], "quality"));
    fix.satellites = fields[7].empty() ? 0 : static_cast<int>(parseLongStrict(fields[7], "satellites"));
    fix.has_hdop = optionalDouble(fields[8], "HDOP", &fix.hdop);
    fix.has_altitude_msl = optionalDouble(fields[9], "altitude", &fix.altitude_msl);
    fix.has_geoid_separation =
        optionalDouble(fields[11], "geoid separation", &fix.geoid_separation);
  } catch (const ProtocolError&) {
    throw;
  }

  if (fix.quality > 0) {
    fix.latitude = coordinate(fields[2], fields[3], true);
    fix.longitude = coordinate(fields[4], fields[5], false);
    fix.has_latitude = true;
    fix.has_longitude = true;
  }
  return fix;
}

GgaFix parseGgaDatagram(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr) {
    throw ProtocolError("GPS datagram data is null");
  }
  std::string text;
  text.reserve(size);
  for (std::size_t index = 0U; index < size; ++index) {
    if (data[index] > 0x7fU) {
      throw ProtocolError("GPS datagram is not ASCII");
    }
    if (data[index] != 0U) {
      text.push_back(static_cast<char>(data[index]));
    }
  }

  std::size_t begin = text.find('$');
  while (begin != std::string::npos) {
    const std::size_t next = text.find('$', begin + 1U);
    const std::string candidate =
        trimNmea(text.substr(begin, next == std::string::npos ? std::string::npos : next - begin));
    if (candidate.size() >= 6U && candidate.compare(3U, 3U, "GGA") == 0) {
      return parseGga(candidate);
    }
    begin = next;
  }
  throw GgaSentenceNotFound();
}

}  // namespace morai_udp_bridge
