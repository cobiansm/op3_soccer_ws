################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(op3_leo)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  robotis_controller_msgs
  roscpp
  sensor_msgs
  std_msgs
  rospy
  cv_bridge
  image_transport
  geometry_msgs
  op3_action_module_msgs
  robotis_math
  op3_walking_module_msgs
)

find_package(Eigen3 REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    robotis_controller_msgs
    sensor_msgs
    std_msgs
    rospy
    cv_bridge
    image_transport
    geometry_msgs
    op3_action_module_msgs
    robotis_math
    op3_walking_module_msgs
    DEPENDS EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(read_write
  src/read_write.cpp
)

add_dependencies(read_write
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(read_write
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

################################################################################
# Install
################################################################################
install(TARGETS read_write
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
