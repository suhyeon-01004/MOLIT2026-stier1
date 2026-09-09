cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Threads REQUIRED)

add_message_files(FILES
  ActuatorCommand.msg
  CompetitionVehicleStatus.msg
  GearCommand.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_udp_bridge_protocol morai_udp_bridge_runtime
            morai_udp_bridge_control_protocol morai_udp_bridge_udp_sender
            morai_udp_bridge_control_watchdog
            morai_udp_bridge_control_sender_config
            morai_udp_bridge_competition_status_protocol
            morai_udp_bridge_gear_change_interlock
  CATKIN_DEPENDS diagnostic_msgs message_runtime roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_udp_bridge_protocol
  src/sensors/protocol.cpp
)
target_compile_features(morai_udp_bridge_protocol PUBLIC cxx_std_14)

add_library(morai_udp_bridge_runtime
  src/sensors/streams.cpp
  src/network/transport.cpp
)
target_compile_features(morai_udp_bridge_runtime PUBLIC cxx_std_14)
target_link_libraries(morai_udp_bridge_runtime
  morai_udp_bridge_protocol
  ${catkin_LIBRARIES}
  Threads::Threads
)
add_dependencies(morai_udp_bridge_runtime ${catkin_EXPORTED_TARGETS})

add_library(morai_udp_bridge_control_protocol
  src/control/control_protocol.cpp
)
target_compile_features(morai_udp_bridge_control_protocol PUBLIC cxx_std_14)

add_library(morai_udp_bridge_competition_status_protocol
  src/sensors/competition_status_protocol.cpp
)
target_compile_features(morai_udp_bridge_competition_status_protocol
  PUBLIC cxx_std_14
)

add_library(morai_udp_bridge_gear_change_interlock
  src/control/gear_change_interlock.cpp
)
target_compile_features(morai_udp_bridge_gear_change_interlock
  PUBLIC cxx_std_14
)

add_library(morai_udp_bridge_udp_sender
  src/network/udp_sender.cpp
)
target_compile_features(morai_udp_bridge_udp_sender PUBLIC cxx_std_14)

add_library(morai_udp_bridge_control_watchdog
  src/control/control_watchdog.cpp
)
target_compile_features(morai_udp_bridge_control_watchdog PUBLIC cxx_std_14)

add_library(morai_udp_bridge_control_sender_config
  src/control/control_sender_config.cpp
)
target_compile_features(morai_udp_bridge_control_sender_config PUBLIC cxx_std_14)
target_link_libraries(morai_udp_bridge_control_sender_config ${catkin_LIBRARIES})

add_executable(morai_udp_bridge_node
  src/nodes/bridge_node.cpp
)
target_compile_features(morai_udp_bridge_node PUBLIC cxx_std_14)
target_link_libraries(morai_udp_bridge_node
  morai_udp_bridge_runtime
  morai_udp_bridge_protocol
  ${catkin_LIBRARIES}
  Threads::Threads
)
add_dependencies(morai_udp_bridge_node ${catkin_EXPORTED_TARGETS})

add_executable(control_sender_node
  src/nodes/control_sender_node.cpp
)
target_compile_features(control_sender_node PUBLIC cxx_std_14)
target_link_libraries(control_sender_node
  morai_udp_bridge_control_protocol
  morai_udp_bridge_control_sender_config
  morai_udp_bridge_control_watchdog
  morai_udp_bridge_gear_change_interlock
  morai_udp_bridge_udp_sender
  ${catkin_LIBRARIES}
)
add_dependencies(control_sender_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(competition_vehicle_status_receiver_node
  src/nodes/competition_vehicle_status_receiver_node.cpp
)
target_compile_features(competition_vehicle_status_receiver_node
  PUBLIC cxx_std_14
)
target_link_libraries(competition_vehicle_status_receiver_node
  morai_udp_bridge_competition_status_protocol
  morai_udp_bridge_runtime
  ${catkin_LIBRARIES}
  Threads::Threads
)
add_dependencies(competition_vehicle_status_receiver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    morai_udp_bridge_protocol
    morai_udp_bridge_runtime
    morai_udp_bridge_control_protocol
    morai_udp_bridge_udp_sender
    morai_udp_bridge_control_watchdog
    morai_udp_bridge_control_sender_config
    morai_udp_bridge_competition_status_protocol
    morai_udp_bridge_gear_change_interlock
    morai_udp_bridge_node
    control_sender_node
    competition_vehicle_status_receiver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/unit/test_autonomous_config_contract.py)

  catkin_add_gtest(test_protocols_cpp test/unit/test_protocols_cpp.cpp)
  if(TARGET test_protocols_cpp)
    target_compile_features(test_protocols_cpp PUBLIC cxx_std_14)
    target_link_libraries(test_protocols_cpp morai_udp_bridge_protocol)
  endif()

  catkin_add_gtest(test_control_protocol test/unit/test_control_protocol.cpp)
  if(TARGET test_control_protocol)
    target_compile_features(test_control_protocol PUBLIC cxx_std_14)
    target_link_libraries(test_control_protocol morai_udp_bridge_control_protocol)
  endif()

  catkin_add_gtest(test_control_watchdog test/unit/test_control_watchdog.cpp)
  if(TARGET test_control_watchdog)
    target_compile_features(test_control_watchdog PUBLIC cxx_std_14)
    target_link_libraries(test_control_watchdog
      morai_udp_bridge_control_watchdog
      morai_udp_bridge_control_protocol
    )
  endif()

  catkin_add_gtest(test_control_sender_config
    test/unit/test_control_sender_config.cpp
  )
  if(TARGET test_control_sender_config)
    target_compile_features(test_control_sender_config PUBLIC cxx_std_14)
    target_link_libraries(test_control_sender_config
      morai_udp_bridge_control_sender_config
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_competition_status_protocol
    test/unit/test_competition_status_protocol.cpp
  )
  if(TARGET test_competition_status_protocol)
    target_compile_features(test_competition_status_protocol PUBLIC cxx_std_14)
    target_link_libraries(test_competition_status_protocol
      morai_udp_bridge_competition_status_protocol
    )
  endif()

  catkin_add_gtest(test_gear_change_interlock
    test/unit/test_gear_change_interlock.cpp
  )
  if(TARGET test_gear_change_interlock)
    target_compile_features(test_gear_change_interlock PUBLIC cxx_std_14)
    target_link_libraries(test_gear_change_interlock
      morai_udp_bridge_gear_change_interlock
    )
  endif()

  # CMake test targets are global across this workspace; vehicle_control
  # already owns test_udp_sender.
  catkin_add_gtest(test_morai_udp_bridge_udp_sender
    test/unit/test_udp_sender.cpp
  )
  if(TARGET test_morai_udp_bridge_udp_sender)
    target_compile_features(test_morai_udp_bridge_udp_sender PUBLIC cxx_std_14)
    target_link_libraries(test_morai_udp_bridge_udp_sender morai_udp_bridge_udp_sender)
    add_custom_target(run_tests_morai_udp_bridge_gtest_test_udp_sender
      DEPENDS run_tests_morai_udp_bridge_gtest_test_morai_udp_bridge_udp_sender)
  endif()
endif()
