cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare a C++ executable
add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)

# Specify include directories
ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  sensor_msgs
)

install(TARGETS
  pointcloud_to_laserscan
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
