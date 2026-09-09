cmake_minimum_required(VERSION 3.0.2)
project(ioniq5_description)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/description/test_vehicle_description.py)
endif()

install(
  DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
