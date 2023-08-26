cmake_minimum_required(VERSION 3.8)
project(miyalab_msg_viewer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
 
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED core highgui)
find_package(cv_bridge REQUIRED)
find_package(miyalab_ros_library REQUIRED)

include_directories(
  include
)
add_library(${PROJECT_NAME} SHARED
  src/imu.cpp
  src/laser_scan.cpp
  src/image.cpp
  src/point_cloud.cpp
  src/point_cloud2.cpp
  src/odometry.cpp
)
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  OpenCV
  cv_bridge
  miyalab_ros_library
)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::OdometryViewer" EXECUTABLE "odometry_msg_viewer_node")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::LaserScanViewer" EXECUTABLE "laser_scan_msg_viewer")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::ImageViewer" EXECUTABLE "image_msg_viewer")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::ImuViewer" EXECUTABLE "imu_msg_viewer")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::PointCloudViewer" EXECUTABLE "point_cloud_viewer")
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "MiYALAB::ROS2::PointCloud2Viewer" EXECUTABLE "point_cloud2_viewer")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# yaml
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()