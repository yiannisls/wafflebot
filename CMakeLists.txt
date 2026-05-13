cmake_minimum_required(VERSION 3.8)
project(wafflebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sofar_lab REQUIRED)

add_executable(waffle_cmd src/waffle_cmd.cpp)
ament_target_dependencies(waffle_cmd
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  sofar_lab
)

install(TARGETS waffle_cmd DESTINATION lib/${PROJECT_NAME})

ament_package()
