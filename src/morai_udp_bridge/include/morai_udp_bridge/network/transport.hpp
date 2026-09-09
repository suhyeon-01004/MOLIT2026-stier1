#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <diagnostic_msgs/DiagnosticStatus.h>

namespace morai_udp_bridge {

class StreamStats {
 public:
  StreamStats(std::string name, std::string bind_ip, std::uint16_t port,
              std::string topic, double stale_timeout, double max_hz);

  void setSocketBuffers(std::size_t requested, std::size_t actual);
  void received(const std::string& sender);
  void rejected();
  void published();
  void parseError();
  void callbackError();
  void setDropped(std::uint64_t count);
  diagnostic_msgs::DiagnosticStatus diagnostic() const;

  const std::string& name() const { return name_; }

 private:
  using Clock = std::chrono::steady_clock;

  const std::string name_;
  const std::string bind_ip_;
  const std::uint16_t port_;
  const std::string topic_;
  const double stale_timeout_;
  const double max_hz_;
  const Clock::time_point started_;

  mutable std::mutex mutex_;
  bool has_last_rx_{false};
  Clock::time_point last_rx_;
  std::string last_sender_;
  std::uint64_t received_{0};
  std::uint64_t rejected_{0};
  std::uint64_t published_{0};
  std::uint64_t parse_errors_{0};
  std::uint64_t callback_errors_{0};
  std::uint64_t dropped_{0};
  std::size_t requested_receive_buffer_{0};
  std::size_t actual_receive_buffer_{0};
  std::deque<Clock::time_point> publish_times_;
};

class UdpWorker {
 public:
  using Callback = std::function<void(const std::uint8_t*, std::size_t)>;

  UdpWorker(std::string bind_ip, std::uint16_t port, Callback callback,
            std::shared_ptr<StreamStats> stats, std::size_t receive_buffer,
            std::string allowed_source_ip = std::string());
  ~UdpWorker();

  UdpWorker(const UdpWorker&) = delete;
  UdpWorker& operator=(const UdpWorker&) = delete;

  void start();
  void close();

 private:
  void run();

  const std::string bind_ip_;
  const std::uint16_t port_;
  const Callback callback_;
  const std::shared_ptr<StreamStats> stats_;
  const std::string allowed_source_ip_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> started_{false};
  std::mutex close_mutex_;
  int socket_fd_{-1};
  std::thread thread_;
};

}  // namespace morai_udp_bridge
