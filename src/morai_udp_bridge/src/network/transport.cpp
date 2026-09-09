#include "morai_udp_bridge/network/transport.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>

namespace morai_udp_bridge {
namespace {

std::string fixed(double value, int precision) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string endpoint(const std::string& ip, std::uint16_t port) {
  std::ostringstream output;
  output << ip << ':' << port;
  return output.str();
}

int createBoundSocket(const std::string& bind_ip, std::uint16_t port,
                      std::size_t requested_receive_buffer,
                      std::size_t* actual_receive_buffer) {
  if (requested_receive_buffer > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("receive buffer is larger than SO_RCVBUF accepts");
  }

  struct addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  hints.ai_flags = AI_PASSIVE;

  const std::string service = std::to_string(port);
  struct addrinfo* addresses = nullptr;
  const char* host = (bind_ip.empty() || bind_ip == "0.0.0.0") ? nullptr : bind_ip.c_str();
  const int lookup = getaddrinfo(host, service.c_str(), &hints, &addresses);
  if (lookup != 0) {
    throw std::runtime_error("cannot resolve UDP bind address " + bind_ip + ": " +
                             gai_strerror(lookup));
  }

  int last_error = EADDRNOTAVAIL;
  int fd = -1;
  for (const struct addrinfo* address = addresses; address != nullptr;
       address = address->ai_next) {
    fd = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (fd < 0) {
      last_error = errno;
      continue;
    }

    const int requested = static_cast<int>(requested_receive_buffer);
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &requested, sizeof(requested)) != 0) {
      last_error = errno;
      ::close(fd);
      fd = -1;
      continue;
    }

    int reported = 0;
    socklen_t option_size = sizeof(reported);
    if (::getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &reported, &option_size) != 0) {
      last_error = errno;
      ::close(fd);
      fd = -1;
      continue;
    }
#ifdef __linux__
    // Linux doubles SO_RCVBUF internally for kernel bookkeeping. Report the
    // effective user-data capacity as half of the kernel-reported value.
    *actual_receive_buffer = static_cast<std::size_t>(reported) / 2U;
#else
    *actual_receive_buffer = static_cast<std::size_t>(reported);
#endif

    struct timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
      last_error = errno;
      ::close(fd);
      fd = -1;
      continue;
    }

    // Deliberately do not set SO_REUSEADDR/SO_REUSEPORT. A second bridge on
    // the same destination must fail instead of silently sharing datagrams.
    if (::bind(fd, address->ai_addr, address->ai_addrlen) == 0) {
      break;
    }
    last_error = errno;
    ::close(fd);
    fd = -1;
  }
  freeaddrinfo(addresses);

  if (fd < 0) {
    throw std::runtime_error("cannot bind UDP socket " + endpoint(bind_ip, port) + ": " +
                             std::strerror(last_error));
  }
  return fd;
}

std::string senderAddress(const struct sockaddr_storage& address) {
  std::array<char, INET6_ADDRSTRLEN> buffer{};
  std::uint16_t port = 0U;
  const void* raw_address = nullptr;
  if (address.ss_family == AF_INET) {
    const auto* ipv4 = reinterpret_cast<const struct sockaddr_in*>(&address);
    raw_address = &ipv4->sin_addr;
    port = ntohs(ipv4->sin_port);
  } else if (address.ss_family == AF_INET6) {
    const auto* ipv6 = reinterpret_cast<const struct sockaddr_in6*>(&address);
    raw_address = &ipv6->sin6_addr;
    port = ntohs(ipv6->sin6_port);
  } else {
    return "unknown:0";
  }
  if (::inet_ntop(address.ss_family, raw_address, buffer.data(), buffer.size()) == nullptr) {
    return "unknown:0";
  }
  return endpoint(buffer.data(), port);
}

std::string senderIp(const struct sockaddr_storage& address) {
  std::array<char, INET6_ADDRSTRLEN> buffer{};
  const void* raw_address = nullptr;
  if (address.ss_family == AF_INET) {
    raw_address = &reinterpret_cast<const struct sockaddr_in*>(&address)->sin_addr;
  } else if (address.ss_family == AF_INET6) {
    raw_address = &reinterpret_cast<const struct sockaddr_in6*>(&address)->sin6_addr;
  } else {
    return std::string();
  }
  if (::inet_ntop(address.ss_family, raw_address, buffer.data(), buffer.size()) == nullptr) {
    return std::string();
  }
  return buffer.data();
}

}  // namespace

StreamStats::StreamStats(std::string name, std::string bind_ip, std::uint16_t port,
                         std::string topic, double stale_timeout, double max_hz)
    : name_(std::move(name)),
      bind_ip_(std::move(bind_ip)),
      port_(port),
      topic_(std::move(topic)),
      stale_timeout_(stale_timeout),
      max_hz_(max_hz),
      started_(Clock::now()) {}

void StreamStats::setSocketBuffers(std::size_t requested, std::size_t actual) {
  std::lock_guard<std::mutex> lock(mutex_);
  requested_receive_buffer_ = requested;
  actual_receive_buffer_ = actual;
}

void StreamStats::received(const std::string& sender) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++received_;
  has_last_rx_ = true;
  last_rx_ = Clock::now();
  last_sender_ = sender;
}

void StreamStats::rejected() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++rejected_;
}

void StreamStats::published() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++published_;
  publish_times_.push_back(Clock::now());
  if (publish_times_.size() > 256U) {
    publish_times_.pop_front();
  }
}

void StreamStats::parseError() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++parse_errors_;
}

void StreamStats::callbackError() {
  std::lock_guard<std::mutex> lock(mutex_);
  // Keep the aggregate parse_errors counter for diagnostics compatibility and
  // expose unexpected callback failures separately as well.
  ++parse_errors_;
  ++callback_errors_;
}

void StreamStats::setDropped(std::uint64_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  dropped_ = count;
}

diagnostic_msgs::DiagnosticStatus StreamStats::diagnostic() const {
  const Clock::time_point now = Clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  const double age = std::chrono::duration<double>(
                         has_last_rx_ ? now - last_rx_ : now - started_)
                         .count();

  auto recent_begin = publish_times_.begin();
  while (recent_begin != publish_times_.end() &&
         std::chrono::duration<double>(now - *recent_begin).count() > 5.0) {
    ++recent_begin;
  }
  double hz = 0.0;
  const auto recent_count = std::distance(recent_begin, publish_times_.end());
  if (recent_count >= 2) {
    const double elapsed =
        std::chrono::duration<double>(publish_times_.back() - *recent_begin).count();
    if (elapsed > 0.0) {
      hz = static_cast<double>(recent_count - 1) / elapsed;
    }
  }

  diagnostic_msgs::DiagnosticStatus status;
  if (!has_last_rx_) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "waiting for UDP packets";
  } else if (age > stale_timeout_) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "UDP stream is stale";
  } else if (max_hz_ > 0.0 && hz > max_hz_ * 1.05) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "publish rate exceeds configured competition limit";
  } else if (requested_receive_buffer_ > 0U &&
             actual_receive_buffer_ < requested_receive_buffer_) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "kernel receive buffer is smaller than requested";
  } else {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "receiving";
  }

  status.name = "morai_udp_bridge: " + name_;
  status.hardware_id = "udp://" + endpoint(bind_ip_, port_);
  status.values = {
      keyValue("topic", topic_),
      keyValue("last_sender", last_sender_.empty() ? "none" : last_sender_),
      keyValue("age_seconds", fixed(age, 3)),
      keyValue("publish_hz_5s", fixed(hz, 2)),
      keyValue("competition_max_hz", fixed(max_hz_, 2)),
      keyValue("received_datagrams", std::to_string(received_)),
      keyValue("rejected_datagrams", std::to_string(rejected_)),
      keyValue("published_messages", std::to_string(published_)),
      keyValue("parse_errors", std::to_string(parse_errors_)),
      keyValue("callback_errors", std::to_string(callback_errors_)),
      keyValue("dropped_frames", std::to_string(dropped_)),
      keyValue("requested_receive_buffer_bytes", std::to_string(requested_receive_buffer_)),
      keyValue("actual_receive_buffer_bytes", std::to_string(actual_receive_buffer_)),
  };
  return status;
}

UdpWorker::UdpWorker(std::string bind_ip, std::uint16_t port, Callback callback,
                     std::shared_ptr<StreamStats> stats, std::size_t receive_buffer,
                     std::string allowed_source_ip)
    : bind_ip_(std::move(bind_ip)),
      port_(port),
      callback_(std::move(callback)),
      stats_(std::move(stats)),
      allowed_source_ip_(std::move(allowed_source_ip)) {
  if (!callback_) {
    throw std::invalid_argument("UDP callback is empty");
  }
  if (!stats_) {
    throw std::invalid_argument("UDP stats object is null");
  }
  if (receive_buffer == 0U) {
    throw std::invalid_argument("receive buffer must be positive");
  }

  std::size_t actual_receive_buffer = 0U;
  socket_fd_ = createBoundSocket(bind_ip_, port_, receive_buffer, &actual_receive_buffer);
  stats_->setSocketBuffers(receive_buffer, actual_receive_buffer);
  if (actual_receive_buffer < receive_buffer) {
    ROS_WARN("%s requested a %zu-byte UDP receive buffer, but the kernel provided %zu bytes",
             stats_->name().c_str(), receive_buffer, actual_receive_buffer);
  }
}

UdpWorker::~UdpWorker() { close(); }

void UdpWorker::start() {
  bool expected = false;
  if (!started_.compare_exchange_strong(expected, true)) {
    throw std::logic_error("UDP worker was started more than once");
  }
  thread_ = std::thread(&UdpWorker::run, this);
}

void UdpWorker::run() {
  std::array<std::uint8_t, 65535U> buffer{};
  while (!stop_requested_.load() && ros::ok()) {
    struct sockaddr_storage sender {};
    socklen_t sender_size = sizeof(sender);
    const ssize_t received = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                        reinterpret_cast<struct sockaddr*>(&sender),
                                        &sender_size);
    if (received < 0) {
      const int error = errno;
      if (stop_requested_.load()) {
        break;
      }
      if (error == EAGAIN || error == EWOULDBLOCK || error == EINTR) {
        continue;
      }
      stats_->callbackError();
      ROS_WARN_THROTTLE(5.0, "%s UDP receive error: %s", stats_->name().c_str(),
                        std::strerror(error));
      break;
    }

    const std::string sender_ip = senderIp(sender);
    if (!allowed_source_ip_.empty() && sender_ip != allowed_source_ip_) {
      stats_->rejected();
      continue;
    }
    stats_->received(senderAddress(sender));
    try {
      callback_(buffer.data(), static_cast<std::size_t>(received));
    } catch (const std::exception& error) {
      stats_->callbackError();
      ROS_WARN_THROTTLE(5.0, "%s callback error: %s", stats_->name().c_str(), error.what());
    } catch (...) {
      stats_->callbackError();
      ROS_WARN_THROTTLE(5.0, "%s callback error: unknown exception",
                        stats_->name().c_str());
    }
  }
}

void UdpWorker::close() {
  std::lock_guard<std::mutex> lock(close_mutex_);
  stop_requested_.store(true);
  if (socket_fd_ >= 0) {
    // Closing the descriptor plus the 200 ms timeout guarantees bounded exit.
    ::shutdown(socket_fd_, SHUT_RD);
    ::close(socket_fd_);
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  socket_fd_ = -1;
}

}  // namespace morai_udp_bridge
