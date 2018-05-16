cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(minimum_distance_plugin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CCD REQUIRED ccd)

find_package(gazebo 7 REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(minimum_distance_plugin SHARED
  minimum_distance_plugin.cpp
  CCDWrapper.cpp)

target_link_libraries(minimum_distance_plugin PRIVATE
  ${CCD_LIBRARIES}
  ${GAZEBO_LIBRARIES})

target_include_directories(minimum_distance_plugin PRIVATE
  ${CCD_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR})

target_compile_options(minimum_distance_plugin PRIVATE -std=c++11)

install(TARGETS minimum_distance_plugin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/static)
