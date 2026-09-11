#pragma once
#include <cmath>
#include <deque>
#include <stdexcept>

namespace state_time {
struct Command { double stamp, angle; };
struct Pose { double x, y, yaw; };

inline void checkAge(double dt) {
  if (!std::isfinite(dt) || dt<0. || dt>.15)
    throw std::invalid_argument("Observation age outside [0,0.15] seconds");
}

inline Pose predictPose(Pose p, double speed, double yaw_rate, double dt) {
  checkAge(dt);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
      !std::isfinite(speed) || std::abs(speed)>50. || !std::isfinite(yaw_rate) || std::abs(yaw_rate)>3.)
    throw std::invalid_argument("Invalid state prediction input");
  // ponytail: short constant-speed/yaw-rate arc, no lateral-slip state or outage prediction.
  const double half=yaw_rate*dt*.5;
  const double distance=speed*dt*(std::abs(half)<1e-8 ? 1. : std::sin(half)/half);
  p.x+=distance*std::cos(p.yaw+half); p.y+=distance*std::sin(p.yaw+half);
  p.yaw=std::atan2(std::sin(p.yaw+2*half),std::cos(p.yaw+2*half));
  return p;
}

inline double commandAt(const std::deque<Command>& history, double stamp) {
  if (!std::isfinite(stamp)) throw std::invalid_argument("Invalid command query time");
  for (auto i=history.rbegin();i!=history.rend();++i)
    if (i->stamp<=stamp) return i->angle;
  throw std::invalid_argument("Sent command history does not cover prediction interval");
}

inline double predictSteering(double angle, double from, double to,
    const std::deque<Command>& history, double delay, double tau, double gain) {
  checkAge(to-from);
  if (!std::isfinite(angle) || !std::isfinite(delay) || delay<0. || delay>.5 ||
      !std::isfinite(tau) || tau<.01 || tau>1. || !std::isfinite(gain) || gain<.5 || gain>1.5)
    throw std::invalid_argument("Invalid steering prediction input");
  double at=from,command=commandAt(history,from-delay)*gain;
  for (const auto& event:history) {
    const double active=event.stamp+delay;
    if (active<=from) continue;
    if (active>to) break;
    angle=command+(angle-command)*std::exp(-(active-at)/tau);
    at=active; command=event.angle*gain;
  }
  return command+(angle-command)*std::exp(-(to-at)/tau);
}
}  // namespace state_time
