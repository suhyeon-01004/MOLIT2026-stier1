cmake_minimum_required(VERSION 3.0.2)
project(morai_kcity_hd_map)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pluginlib
  roscpp
  rospy
  rviz
  std_msgs
  visualization_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_message_files(FILES RouteSpeedLimit.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_kcity_status_panel
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs pluginlib roscpp rospy
                 rviz std_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_kcity_status_panel
  include/morai_kcity_hd_map/localization_status_panel.hpp
  src/localization_status_panel.cpp
)
target_compile_features(morai_kcity_status_panel PUBLIC cxx_std_14)
target_link_libraries(morai_kcity_status_panel
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/build_lanelet2.py
    scripts/map_clearance.py
    scripts/rddf_lanelet_route_node.py
    scripts/rviz_visualizer.py
    scripts/validate_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# mgeo_common is imported by the executable scripts. A catkin Python relay is
# executable-only and does not export imported symbols, so keep this module as
# a regular file in both the devel and install spaces.
file(MAKE_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
)
configure_file(
  scripts/mgeo_common.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/mgeo_common.py
  COPYONLY
)
configure_file(
  scripts/crosswalk_geometry.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/crosswalk_geometry.py
  COPYONLY
)

install(
  TARGETS morai_kcity_status_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES scripts/mgeo_common.py scripts/crosswalk_geometry.py
        rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Installed Python executables import these sibling modules at runtime.
install(
  FILES scripts/mgeo_common.py scripts/crosswalk_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch map raw_mgeo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
