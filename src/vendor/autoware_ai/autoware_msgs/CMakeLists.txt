cmake_minimum_required(VERSION 3.0.2)
project(autoware_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)
# Original definitions, only the MPC dependency closure is built.
add_message_files(FILES Lane.msg Waypoint.msg DTLane.msg WaypointState.msg VehicleStatus.msg Gear.msg ControlCommand.msg ControlCommandStamped.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs std_msgs)
