cmake_minimum_required(VERSION 3.5)
project(rviz_doodles)

#=============
# Setup
#=============

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Emulate colcon
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(flock_vlam_msgs_DIR "${PROJECT_SOURCE_DIR}/../../../install/flock_vlam_msgs/share/flock_vlam_msgs/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Local includes
include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Debugging: set _dump_all_variables to true
set(_dump_all_variables false)
if(_dump_all_variables)
  get_cmake_property(_variable_names VARIABLES)
  list(SORT _variable_names)
  foreach(_variable_name ${_variable_names})
    message(STATUS "${_variable_name}=${${_variable_name}}")
  endforeach()
endif()

#=============
# ${PROJECT_NAME} node
#=============

add_executable(${PROJECT_NAME}
  src/rviz_doodles.cpp
  )

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
  )

#=============
# Install
#=============

# Install targets
install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install various directories
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#=============
# Run ament macros
#=============

ament_package()
