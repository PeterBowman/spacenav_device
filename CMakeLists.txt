cmake_minimum_required(VERSION 3.8)
project(spacenav_device)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(spacenav_device src/spacenav_device.cpp)
ament_target_dependencies(spacenav_device rclcpp geometry_msgs)

install(TARGETS
  spacenav_device
  DESTINATION lib/${PROJECT_NAME})

ament_package()
