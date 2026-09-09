#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace morai_udp_bridge {

class UdpSender {
 public:
  UdpSender(const std::string& destination_ip, std::uint16_t destination_port);
  ~UdpSender();

  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;

  void send(const std::uint8_t* data, std::size_t size);

 private:
  int socket_fd_{-1};
  struct sockaddr_in destination_ {};
};

}  // namespace morai_udp_bridge
