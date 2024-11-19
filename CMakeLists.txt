cmake_minimum_required(VERSION 3.0.2)
project(trajectory_saver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mongodb_store
  message_generation
  geometry_msgs
  trajectory_saver_msg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_saver
  # CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/trajectory_saver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)