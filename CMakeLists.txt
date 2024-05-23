cmake_minimum_required(VERSION 3.15)
project(dsol)

set(CMAKE_CXX_STANDARD 17)
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )

include_directories(.)
list(PREPEND CMAKE_MODULE_PATH "/opt/ros/noetic/share/sophus/cmake" )

option(BUILD_ROS "Build ros" ON)
option(BUILD_PLAY "Build play" OFF)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_BENCHMARK "Build benchmarks" OFF)
option(BUILD_VECTORIZE "Build with -march=native" ON)
set(SOPHUS_INCLUDE_DIR_HINTS /opt/ros/noetic/include/)
set(SOPHUS_INCLUDE_DIR /opt/ros/noetic/include/)
set(CC_TARGET_PREFIX sv)
include(CMakeHelpers)

add_library(sv_options INTERFACE)
include(CompilerWarnings)
enable_warnings(sv_options)
include(Sanitizers)
enable_sanitizers(sv_options)

find_package(
  catkin QUIET
  COMPONENTS roscpp
             tf2_ros
             tf2_eigen
             cv_bridge
             image_transport
             nav_msgs
             sensor_msgs
             visualization_msgs)

find_package(TBB REQUIRED)
find_package(fmt REQUIRED)
find_package(glog REQUIRED)
add_subdirectory(third_party)
#find_package(absl REQUIRED)
find_package(Boost REQUIRED) # circular buffer

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(xtensor) # reading npy
set(xtensor_FOUND 1)
include_directories(/opt/ros/noetic/include)
find_package(GTest REQUIRED)
set(BENCHMARK_DIR /usr/include)
find_package(Benchmark REQUIRED)

if(NOT catkin_FOUND)
  set(BUILD_ROS OFF)
  message(WARN "BUILD_ROS set to OFF because catkin not found.")
endif()

if(BUILD_ROS)
  catkin_package()
endif()

enable_testing()
add_subdirectory(sv)
