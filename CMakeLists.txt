cmake_minimum_required(VERSION 3.0.2)
project(multi_quadcopter_control)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Set the build type. Options are:
## Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
## Debug          : w/ debug symbols, w/o optimization
## Release        : w/o debug symbols, w/ optimization
## RelWithDebInfo : w/ debug symbols, w/ optimization
## MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

## General optimizations (balanced for performance and stability)
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_compile_options("-O2" "-march=native")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  genmsg
  tf2
  tf2_geometry_msgs
  geometry_msgs
  message_filters
)

add_service_files(
  FILES
  WaypointService.srv
)

generate_messages(
  DEPENDENCIES
  mavros_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES multi_quadcopter_control
 CATKIN_DEPENDS mavros_msgs roscpp std_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Apply specific optimizations for performance-critical targets
set(CRITICAL_OPT_FLAGS "-O3 -march=native -funroll-loops")

add_executable(multi_mid_level_cont_node src/mid_level_cont_node.cpp src/mid_level_cont.cpp)
target_link_libraries(multi_mid_level_cont_node ${catkin_LIBRARIES})
add_dependencies(multi_mid_level_cont_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(multi_mid_level_cont_node PROPERTIES COMPILE_FLAGS "${CRITICAL_OPT_FLAGS}")

add_executable(multi_low_level_cont_node src/low_level_cont_node.cpp src/low_level_cont.cpp)
target_link_libraries(multi_low_level_cont_node ${catkin_LIBRARIES})
add_dependencies(multi_low_level_cont_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(multi_low_level_cont_node PROPERTIES COMPILE_FLAGS "${CRITICAL_OPT_FLAGS}")

## Other executables
add_executable(multi_trajectory_generator_node src/trajectory_generator_node.cpp src/trajectory_generator.cpp)
target_link_libraries(multi_trajectory_generator_node ${catkin_LIBRARIES})
add_dependencies(multi_trajectory_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multi_waypoint_server_node src/waypoint_server_node.cpp src/waypoint_server.cpp)
target_link_libraries(multi_waypoint_server_node ${catkin_LIBRARIES})
add_dependencies(multi_waypoint_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multi_eom src/eom.cpp)
target_link_libraries(multi_eom ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(multi_eom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(multi_rviz_data_handler src/rviz_data_handler.cpp)
target_link_libraries(multi_rviz_data_handler ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS
  multi_trajectory_generator_node
  multi_waypoint_server_node
  multi_mid_level_cont_node
  multi_low_level_cont_node
  multi_eom
  multi_rviz_data_handler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)