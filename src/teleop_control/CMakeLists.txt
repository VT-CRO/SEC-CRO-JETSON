cmake_minimum_required(VERSION 3.8)
project(teleop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teleop_twist_keyboard REQUIRED)

add_executable(teleop_mapper src/teleop_mapper.cpp)
ament_target_dependencies(teleop_mapper rclcpp geometry_msgs std_msgs)

install(TARGETS teleop_mapper
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
