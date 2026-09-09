# Stamp and frame identify the local path / localization sample used.
std_msgs/Header header
string source_link_id
bool high_speed_zone
# Finite operational ceiling: 60 normally; chosen test ceiling on unlimited track.
float64 current_limit_mps
# Map-only target envelope, including advance braking for the next lower limit.
# Curvature, tracking and vehicle constraints are applied downstream.
float64 target_limit_mps
