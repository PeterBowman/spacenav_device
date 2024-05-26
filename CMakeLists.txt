cmake_minimum_required(VERSION 3.8)
project(spacenav_device)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include directories
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(spacenav_device src/spacenav_device.cpp)

ament_target_dependencies(spacenav_device rclcpp geometry_msgs tf2 tf2_geometry_msgs orocos_kdl)
target_link_libraries(spacenav_device ${orocos_kdl_LIBRARIES})
target_include_directories(spacenav_device PUBLIC ${orocos_kdl_INCLUDE_DIRS})

install(TARGETS
  spacenav_device
  DESTINATION lib/${PROJECT_NAME})

ament_package()
