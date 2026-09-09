#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "morai_udp_bridge/sensors/protocol.hpp"

namespace morai_udp_bridge {
namespace {

void appendLe32(std::vector<std::uint8_t>* output, std::uint32_t value) {
  for (std::size_t index = 0U; index < 4U; ++index) {
    output->push_back(static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU));
  }
}

void appendLeDouble(std::vector<std::uint8_t>* output, double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "double must be binary64");
  std::memcpy(&bits, &value, sizeof(bits));
  for (std::size_t index = 0U; index < 8U; ++index) {
    output->push_back(static_cast<std::uint8_t>((bits >> (8U * index)) & 0xffU));
  }
}

std::vector<std::uint8_t> legacyCamera(std::uint32_t index,
                                       const std::vector<std::uint8_t>& payload,
                                       bool final) {
  constexpr std::size_t kCapacity = 64987U;
  std::vector<std::uint8_t> raw{'M', 'O', 'R'};
  appendLe32(&raw, index);
  appendLe32(&raw, static_cast<std::uint32_t>(payload.size()));
  raw.insert(raw.end(), payload.begin(), payload.end());
  raw.resize(11U + kCapacity, 0U);
  raw.push_back(final ? 'E' : 'A');
  raw.push_back('I');
  return raw;
}

std::vector<std::uint8_t> timestampedCamera(std::uint32_t sec, std::uint32_t nsec,
                                            std::uint32_t index,
                                            const std::vector<std::uint8_t>& payload,
                                            bool final) {
  constexpr std::size_t kCapacity = 64979U;
  std::vector<std::uint8_t> raw{'M', 'O', 'R'};
  appendLe32(&raw, sec);
  appendLe32(&raw, nsec);
  appendLe32(&raw, index);
  appendLe32(&raw, static_cast<std::uint32_t>(payload.size()));
  raw.insert(raw.end(), payload.begin(), payload.end());
  raw.resize(19U + kCapacity, 0U);
  raw.push_back(final ? 'E' : 'A');
  raw.push_back('I');
  return raw;
}

std::vector<std::uint8_t> imuPacket(bool timestamped,
                                    const std::array<double, 10>& values,
                                    std::uint32_t sec = 0U, std::uint32_t nsec = 0U) {
  const std::string header = "#IMUData$";
  std::vector<std::uint8_t> raw(header.begin(), header.end());
  appendLe32(&raw, 80U);
  raw.insert(raw.end(), 12U, 0U);
  if (timestamped) {
    appendLe32(&raw, sec);
    appendLe32(&raw, nsec);
  }
  for (const double value : values) {
    appendLeDouble(&raw, value);
  }
  raw.push_back('\r');
  raw.push_back('\n');
  return raw;
}

std::string nmea(const std::string& body) {
  std::uint8_t checksum = 0U;
  for (const unsigned char character : body) {
    checksum ^= character;
  }
  static const char* digits = "0123456789ABCDEF";
  std::string sentence = "$" + body + "*";
  sentence.push_back(digits[(checksum >> 4U) & 0x0fU]);
  sentence.push_back(digits[checksum & 0x0fU]);
  return sentence;
}

TEST(CameraProtocol, LegacySingleChunk) {
  CameraFrameAssembler assembler;
  const std::vector<std::uint8_t> jpeg{0xffU, 0xd8U, 'h', 'e', 'l', 'l', 'o', 0xffU,
                                       0xd9U};
  CameraPacket packet = parseCameraPacket(legacyCamera(0U, jpeg, true));
  EXPECT_EQ(PacketLayout::kLegacy, packet.layout);
  AssembledFrame frame;
  ASSERT_TRUE(assembler.add(std::move(packet), &frame));
  EXPECT_EQ(jpeg, frame.jpeg);
  EXPECT_FALSE(frame.has_timestamp);
}

TEST(CameraProtocol, TimestampedReorderedChunks) {
  CameraFrameAssembler assembler;
  CameraPacket second = parseCameraPacket(timestampedCamera(
      12U, 34U, 1U, {'w', 'o', 'r', 'l', 'd', 0xffU, 0xd9U}, true));
  CameraPacket first = parseCameraPacket(
      timestampedCamera(12U, 34U, 0U, {0xffU, 0xd8U, 'h', 'e', 'l', 'l', 'o', ' '}, false));
  AssembledFrame frame;
  EXPECT_FALSE(assembler.add(std::move(second), &frame));
  ASSERT_TRUE(assembler.add(std::move(first), &frame));
  const std::vector<std::uint8_t> expected{0xffU, 0xd8U, 'h', 'e', 'l', 'l', 'o', ' ',
                                           'w',   'o',   'r', 'l', 'd', 0xffU, 0xd9U};
  EXPECT_EQ(expected, frame.jpeg);
  EXPECT_TRUE(frame.has_timestamp);
  EXPECT_EQ(12U, frame.sec);
  EXPECT_EQ(34U, frame.nsec);
  EXPECT_EQ(2U, frame.packet_count);
}

TEST(CameraProtocol, IncompleteFrameDropsOnNewTimestamp) {
  CameraFrameAssembler assembler;
  AssembledFrame frame;
  EXPECT_FALSE(assembler.add(
      parseCameraPacket(timestampedCamera(1U, 0U, 0U, {0xffU, 0xd8U, 'o', 'l', 'd'}, false)),
      &frame));
  EXPECT_TRUE(assembler.add(parseCameraPacket(timestampedCamera(
                                2U, 0U, 0U, {0xffU, 0xd8U, 'n', 'e', 'w', 0xffU, 0xd9U}, true)),
                            &frame));
  EXPECT_EQ(1U, assembler.droppedFrames());
}

TEST(CameraProtocol, RejectsInvalidTail) {
  auto raw = timestampedCamera(12U, 34U, 0U, {0xffU, 0xd8U, 0xffU, 0xd9U}, true);
  raw[raw.size() - 2U] = 'X';
  raw[raw.size() - 1U] = 'X';
  EXPECT_THROW(parseCameraPacket(raw), ProtocolError);
}

TEST(CameraProtocol, SupportsOneBasedChunkIndexes) {
  CameraFrameAssembler assembler;
  AssembledFrame frame;
  ASSERT_TRUE(assembler.add(parseCameraPacket(legacyCamera(
                                1U, {0xffU, 0xd8U, 'x', 0xffU, 0xd9U}, true)),
                            &frame));
  EXPECT_EQ(1U, frame.packet_count);
}

class ImuProtocol : public ::testing::Test {
 protected:
  const std::array<double, 10> values_{{1.0, 0.1, 0.2, 0.3, 1.1,
                                        1.2, 1.3, 2.1, 2.2, 2.3}};
};

TEST_F(ImuProtocol, Legacy) {
  const ImuSample parsed = parseImuPacket(imuPacket(false, values_));
  EXPECT_EQ(PacketLayout::kLegacy, parsed.layout);
  EXPECT_DOUBLE_EQ(1.0, parsed.orientation_w);
  EXPECT_FALSE(parsed.has_timestamp);
}

TEST_F(ImuProtocol, Timestamped) {
  const ImuSample parsed = parseImuPacket(imuPacket(true, values_, 123U, 456U));
  EXPECT_EQ(PacketLayout::kTimestamped, parsed.layout);
  EXPECT_TRUE(parsed.has_timestamp);
  EXPECT_EQ(123U, parsed.sec);
  EXPECT_EQ(456U, parsed.nsec);
  EXPECT_DOUBLE_EQ(2.3, parsed.linear_acceleration_z);
}

TEST_F(ImuProtocol, RejectsBadHeader) {
  std::vector<std::uint8_t> raw(115U, 'X');
  EXPECT_THROW(parseImuPacket(raw), ProtocolError);
}

TEST_F(ImuProtocol, IgnoresVersionSpecificDataLengthField) {
  auto raw = imuPacket(false, values_);
  raw[9U] = 81U;
  const ImuSample parsed = parseImuPacket(raw);
  EXPECT_DOUBLE_EQ(1.0, parsed.orientation_w);
  EXPECT_DOUBLE_EQ(2.3, parsed.linear_acceleration_z);
}

TEST_F(ImuProtocol, IgnoresPopulatedAuxiliaryData) {
  auto raw = imuPacket(true, values_, 123U, 456U);
  raw[13U] = 1U;
  raw[18U] = 0xa5U;
  const ImuSample parsed = parseImuPacket(raw);
  EXPECT_TRUE(parsed.has_timestamp);
  EXPECT_DOUBLE_EQ(1.0, parsed.orientation_w);
  EXPECT_DOUBLE_EQ(2.3, parsed.linear_acceleration_z);
}

TEST_F(ImuProtocol, RejectsTrailingBytes) {
  auto raw = imuPacket(false, values_);
  raw.push_back('x');
  EXPECT_THROW(parseImuPacket(raw), ProtocolError);
}

TEST_F(ImuProtocol, RejectsNonFiniteValues) {
  auto values = values_;
  values[4] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(parseImuPacket(imuPacket(false, values)), ProtocolError);
}

TEST(NmeaProtocol, ParsesGga) {
  const GgaFix parsed = parseGga(
      nmea("GPGGA,114455,3735.0079,N,12701.6446,E,1,06,7.9,48.8,M,19.6,M,,"));
  EXPECT_TRUE(parsed.valid());
  EXPECT_NEAR(37.583465, parsed.latitude, 1e-6);
  EXPECT_NEAR(127.02741, parsed.longitude, 1e-6);
  EXPECT_NEAR(68.4, parsed.altitudeEllipsoid(), 1e-12);
}

TEST(NmeaProtocol, FindsGgaAfterRmc) {
  const std::string rmc =
      nmea("GPRMC,114455,A,3735.0079,N,12701.6446,E,0.0,121.6,110706,003.3,E");
  const std::string gga =
      nmea("GPGGA,114455,3735.0079,N,12701.6446,E,1,06,7.9,48.8,M,19.6,M,,");
  const std::string text = rmc + "\r\n" + gga + "\r\n";
  const GgaFix parsed = parseGgaDatagram(
      reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
  EXPECT_EQ(1, parsed.quality);
}

TEST(NmeaProtocol, IdentifiesDatagramWithoutGga) {
  const std::string rmc =
      nmea("GPRMC,114455,A,3735.0079,N,12701.6446,E,0.0,121.6,110706,003.3,E");
  EXPECT_THROW(
      parseGgaDatagram(
          reinterpret_cast<const std::uint8_t*>(rmc.data()), rmc.size()),
      GgaSentenceNotFound);
}

TEST(NmeaProtocol, RejectsBadChecksum) {
  EXPECT_THROW(parseGga("$GPGGA,1,,,,,0,0,,,,,,,*00"), ProtocolError);
}

TEST(NmeaProtocol, RejectsMissingChecksum) {
  EXPECT_THROW(parseGga("$GPGGA,114455,3735.0,N,12701.0,E,1,06,1.0,1.0,M,0.0,M,,"),
               ProtocolError);
}

TEST(NmeaProtocol, RejectsInvalidLatitudeDirection) {
  EXPECT_THROW(parseGga(nmea("GPGGA,114455,3735.0,E,12701.0,E,1,06,1.0,1.0,M,0.0,M,,")),
               ProtocolError);
}

TEST(NmeaProtocol, ParsesBlackoutAsNoFix) {
  const GgaFix parsed = parseGga(nmea("GPGGA,114455,,,,,0,00,99.9,,,,,,"));
  EXPECT_FALSE(parsed.valid());
  EXPECT_FALSE(parsed.has_latitude);
}

TEST(NmeaProtocol, RejectsNonAsciiDatagram) {
  const std::vector<std::uint8_t> raw{'$', 'G', 'P', 0xffU};
  EXPECT_THROW(parseGgaDatagram(raw), ProtocolError);
}

}  // namespace
}  // namespace morai_udp_bridge

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
