cmake_minimum_required(VERSION 3.10)

project(third_party LANGUAGES C CXX)

set(ABSL_USE_GOOGLETEST_HEAD ON)
set(CMAKE_CXX_STANDARD 17)
set(ABSL_BUILD_TESTING OFF)
set(ABSL_PROPAGATE_CXX_STD ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(abseil-cpp)

set(BUILD_TESTS OFF)
set(DDOWNLOAD_GTEST OFF)
add_subdirectory(xtensor)

