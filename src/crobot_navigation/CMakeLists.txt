cmake_minimum_required(VERSION 3.8)
project(crobot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  behaviortree_cpp
  behaviortree_ros2
  yaml-cpp
  tf2
  tf2_geometry_msgs
  control_msgs
  apriltag_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
add_executable(crobot_navigation_node 
  src/behavior_node.cpp 
  src/go_to_position.cpp
  src/start.cpp
  src/set_beacon.cpp
  src/set_bin_intake.cpp
  src/set_chassis_velocity.cpp
  src/apriltag_id_subscriber.cpp
)
ament_target_dependencies(crobot_navigation_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(crobot_navigation_node ${YAML_CPP_LIBRARIES})

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

install(
  DIRECTORY tree launch config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/crobot_navigation
)

install(TARGETS crobot_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
