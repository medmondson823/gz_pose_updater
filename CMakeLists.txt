cmake_minimum_required(VERSION 3.10)
project(gz_pose_updater)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(PoseUpdaterPlugin SHARED src/PoseUpdaterPlugin.cpp)
ament_target_dependencies(PoseUpdaterPlugin
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  gz-sim8
)

# Export plugin to lib directory
install(TARGETS PoseUpdaterPlugin
  LIBRARY DESTINATION lib
)

ament_package()

