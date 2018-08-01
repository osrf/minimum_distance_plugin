cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)

project(minimum_distance_plugin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CCD REQUIRED ccd)

find_package(gazebo 7 QUIET)
if (gazebo_FOUND)
  message(STATUS "gazebo7 found!")
else()
  message(STATUS "gazebo7 not found, searching for gazebo8")
  find_package(gazebo 8 QUIET)
  if (gazebo_FOUND)
    message(STATUS "gazebo8 found!")
  else()
    message(STATUS "gazebo8 not found, searching for gazebo9")
    find_package(gazebo 9 QUIET)
    if (gazebo_FOUND)
      message(STATUS "gazebo9 found!")
    else()
      message(STATUS "gazebo9 not found, searching for gazebo10")
      find_package(gazebo 10 QUIET)
    endif()
  endif()
endif()
if (NOT gazebo_FOUND)
  message(FATAL_ERROR "Could not find a version of gazebo >= 7")
endif()

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
