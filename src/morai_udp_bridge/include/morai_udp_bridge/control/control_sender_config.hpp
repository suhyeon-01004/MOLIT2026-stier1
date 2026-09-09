#pragma once

#include <ros/duration.h>

namespace morai_udp_bridge {

float validatedSafeBrakeCommand(double safe_brake_command);
ros::WallDuration controlSendPeriodFromRate(double send_rate_hz);

}  // namespace morai_udp_bridge
