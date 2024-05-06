cmake_minimum_required(VERSION 3.8)
project(miyalab_interfaces)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
  msg/CameraParameter.msg
  msg/Polar.msg
  msg/PointCloudPolar.msg
  msg/RobotState.msg
  msg/Obstacle.msg
  msg/ObstacleArray.msg
  msg/Vector3DArray.msg
  msg/ComputerStatus.msg
)
set(srv_files
  srv/SetInt32.srv
  srv/SetInt64.srv
  srv/SetFloat32.srv
  srv/SetFloat64.srv
  srv/SetString.srv
  srv/SetPose.srv
  srv/ResetRobotState.srv
  srv/GetCameraParameter.srv
  srv/GetComputerInfo.srv
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  #${act_files}
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()