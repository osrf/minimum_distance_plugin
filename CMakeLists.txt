cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(minimum_distance_plugin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CCD REQUIRED ccd)

find_package(gazebo 7 REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(minimum_distance_plugin
  minimum_distance_plugin.cpp
  CCDWrapper.cpp)

target_link_libraries(minimum_distance_plugin PRIVATE
  ${CCD_LIBRARIES}
  ${GAZEBO_LIBRARIES})

target_include_directories(minimum_distance_plugin PRIVATE
  ${CCD_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(minimum_distance_plugin PRIVATE
  cxx_aggregate_default_initializers)

