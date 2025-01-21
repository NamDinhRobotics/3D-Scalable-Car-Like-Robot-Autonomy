cmake_minimum_required(VERSION 3.0.2)
project(base_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
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

add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})

add_executable(manual_ps3 src/manual_ps3.cpp)
target_link_libraries(manual_ps3 ${catkin_LIBRARIES})

add_executable(pure_manual src/pure_manual.cpp)
target_link_libraries(pure_manual ${catkin_LIBRARIES})
