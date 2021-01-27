cmake_minimum_required(VERSION 3.10.2)
project(grad_field_local_planner)

# Finding CXXTest for the unittests:
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cxxtest-4.3)
find_package(CxxTest REQUIRED)

# Enable testing to set up CTEST to use the tests of CXXTest and
# adding the include directories of CXXTest to be able to use the headers.
if (CXXTEST_FOUND)
  include_directories(${CXXTEST_INCLUDE_DIR})
  enable_testing()
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES grad_field_local_planner
#  CATKIN_DEPENDS base_local_planner costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp rospy sensor_msgs std_msgs tf2
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

## add subdirectories: ##
add_subdirectory(src)
if (CXXTEST_FOUND)
  add_subdirectory(tests)
endif()