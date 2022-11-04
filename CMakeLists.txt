cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD COLCON)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")

  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_DOUBLE_RCVBUF "Enable double size of RCVBUF" OFF)
if(${ENABLE_DOUBLE_RCVBUF})
  add_definitions("-DENABLE_DOUBLE_RCVBUF")
endif(${ENABLE_DOUBLE_RCVBUF})

option(ENABLE_WAIT_IF_QUEUE_EMPTY "Enable waiting for a while in handle thread if the queue is empty" OFF)
if(${ENABLE_WAIT_IF_QUEUE_EMPTY})
  add_definitions("-DENABLE_WAIT_IF_QUEUE_EMPTY")
endif(${ENABLE_WAIT_IF_QUEUE_EMPTY})

option(ENABLE_STAMP_WITH_LOCAL "Enable stamp point cloud with local time" OFF)
if(${ENABLE_STAMP_WITH_LOCAL})
  add_definitions("-DENABLE_STAMP_WITH_LOCAL")
endif(${ENABLE_STAMP_WITH_LOCAL})

option(ENABLE_SOURCE_PACKET_LEGACY "Enable ROS Source of MSOP/DIFOP Packet v1.3.x" OFF)
if(${ENABLE_SOURCE_PACKET_LEGACY})
  add_definitions("-DENABLE_SOURCE_PACKET_LEGACY")
endif(${ENABLE_SOURCE_PACKET_LEGACY})

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

add_definitions(-std=c++17)
add_compile_options(-Wall)

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
  add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
  add_definitions(-DPOINT_TYPE_XYZIRT)
endif()

message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Dependencies Setup
#========================


#ROS2#
find_package(rclcpp REQUIRED)

add_definitions(-DROS2_FOUND)
include_directories(${rclcpp_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rslidar_msg REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================

add_executable(rslidar_sdk_node
               node/rslidar_sdk_node.cpp
               src/manager/node_manager.cpp)

target_link_libraries(rslidar_sdk_node
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES})

add_library(rslidar_sdk_component SHARED
    node/rslidar_sdk_component.cpp
    src/manager/node_manager.cpp
)

target_compile_definitions(rslidar_sdk_component
    PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(rslidar_sdk_component
  ${YAML_CPP_LIBRARIES}
  ${rs_driver_LIBRARIES}
)

ament_target_dependencies(rslidar_sdk_component
  rclcpp
  std_msgs
  sensor_msgs
  rslidar_msg
  rclcpp_components
)

rclcpp_components_register_nodes(rslidar_sdk_component "rslidar_driver::RslidarSdkComponent")

# Install components
install(TARGETS rslidar_sdk_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(rslidar_sdk_node
  rclcpp
  std_msgs
  sensor_msgs
  rslidar_msg)

install(TARGETS
  rslidar_sdk_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_libraries(rslidar_sdk_component)
ament_package()

