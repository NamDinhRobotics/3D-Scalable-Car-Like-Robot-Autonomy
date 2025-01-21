cmake_minimum_required(VERSION 3.0.2)
project(cubic_spline)

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
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cubic_spline
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs visualization_msgs
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


add_executable(spline_fit_rviz src/spline_fit_rviz.cpp)
target_link_libraries(spline_fit_rviz ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(spline_circle src/spline_circle.cpp
        include/spline_rw.h)
target_link_libraries(spline_circle ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(load_path src/load_path_orin.cpp)
target_link_libraries(load_path ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(load_path_inter src/load_path_inter.cpp)
target_link_libraries(load_path_inter ${catkin_LIBRARIES} Eigen3::Eigen)

catkin_install_python(PROGRAMS
  scripts/spline_fit_rviz.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
