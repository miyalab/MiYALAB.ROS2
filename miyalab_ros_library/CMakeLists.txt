cmake_minimum_required(VERSION 3.8)
project(miyalab_ros_library)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)
add_library(${PROJECT_NAME} SHARED
  src/type_converter.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()