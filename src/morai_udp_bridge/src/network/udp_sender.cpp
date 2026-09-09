#include "morai_udp_bridge/network/udp_sender.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace morai_udp_bridge {
namespace {

std::runtime_error socketError(const std::string& operation) {
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

}  // namespace

UdpSender::UdpSender(const std::string& destination_ip,
                     std::uint16_t destination_port) {
  if (destination_port == 0U) {
    throw std::invalid_argument("UDP destination port must be in 1..65535");
  }

  destination_.sin_family = AF_INET;
  destination_.sin_port = htons(destination_port);
  const int address_result =
      ::inet_pton(AF_INET, destination_ip.c_str(), &destination_.sin_addr);
  if (address_result != 1) {
    throw std::invalid_argument("UDP destination is not a valid IPv4 address");
  }

  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw socketError("failed to create UDP socket");
  }
}

UdpSender::~UdpSender() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
}

void UdpSender::send(const std::uint8_t* data, std::size_t size) {
  if (data == nullptr && size > 0U) {
    throw std::invalid_argument("UDP payload is null");
  }

  ssize_t sent = -1;
  do {
    sent = ::sendto(socket_fd_, data, size, 0,
                    reinterpret_cast<const struct sockaddr*>(&destination_),
                    sizeof(destination_));
  } while (sent < 0 && errno == EINTR);

  if (sent < 0) {
    throw socketError("failed to send UDP datagram");
  }
  if (static_cast<std::size_t>(sent) != size) {
    throw std::runtime_error("UDP datagram was only partially sent");
  }
}

}  // namespace morai_udp_bridge
