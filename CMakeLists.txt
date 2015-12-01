cmake_minimum_required(VERSION 2.8.3)
project(sonar)

# C++ Compiler Flags
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(GCC_VERSION VERSION_EQUAL 4.7)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

# Find Packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
  cv_bridge
  image_transport
  nav_msgs
)

# Config Files
generate_dynamic_reconfigure_options(
  cfg/Pipeline.cfg
)

# Service Files
add_service_files(
  FILES
  toggle.srv
  sonar_pixel.srv
)

# Message files
add_message_files(
    FILES
    sonar_object.msg
    sonar_objects.msg
    camera_mapping.msg
)

# Generate Messages
generate_messages(
  DEPENDENCIES
  sensor_msgs
  nav_msgs
  std_msgs
)

# Hackjob for mac
if(APPLE)
  catkin_package(
    CATKIN_DEPENDS nav_msgs nodelet roscpp rospy sensor_msgs std_msgs tf message_runtime
    DEPENDS system_lib
  )
  RETURN()
endif()

# Catkin depends
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sonar
  CATKIN_DEPENDS nav_msgs nodelet roscpp rospy sensor_msgs std_msgs tf message_runtime
  DEPENDS system_lib
)

link_directories(
  lib
)

include_directories(
  include
  colormaps
  scripts
  srv
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(sonar
  src/node.cpp
)

add_dependencies(sonar
    ${PROJECT_NAME}_gencpp
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_gensrv
    ${PROJECT_NAME}_genmsg
    sonar_generate_messages_cpp
)

target_link_libraries(sonar
  bvtsdk
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)