cmake_minimum_required(VERSION 3.8)
project(crobot_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(crobot_behavior_lib SHARED
  src/NavigationServer.cpp
  src/action_nodes/go_to_pose.cpp
  src/action_nodes/sweeper_controls.cpp
  src/action_nodes/press_button.cpp
  src/action_nodes/update_footprint.cpp
  src/action_nodes/turn_crank.cpp
  src/action_nodes/embedded_mode.cpp
  # src/action_nodes/timer.cpp
  src/action_nodes/get_that_bag.cpp
  src/action_nodes/wait_for_light.cpp
  src/action_nodes/flag_dropper.cpp
)

ament_target_dependencies(crobot_behavior_lib
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  geometry_msgs
  std_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

target_include_directories(crobot_behavior_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


add_executable(bt_launch
  launch/bt_launch.cpp
)

target_link_libraries(bt_launch
  crobot_behavior_lib
)

ament_target_dependencies(bt_launch
  rclcpp
)

install(
  TARGETS
    crobot_behavior_lib
  EXPORT export_crobot_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    bt_launch
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_crobot_behavior HAS_LIBRARY_TARGET)

ament_package()