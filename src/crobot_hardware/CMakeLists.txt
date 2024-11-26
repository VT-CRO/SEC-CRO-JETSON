cmake_minimum_required(VERSION 3.8)
project(crobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  crobot_hardware
  SHARED
  src/crobot_system.cpp
  src/serial_comm.cpp
  src/crobot_hardware.cpp
)
target_compile_features(crobot_hardware PUBLIC cxx_std_17)
target_include_directories(crobot_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/crobot_hardware>
)
ament_target_dependencies(
  crobot_hardware PUBLIC
  ${INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface crobot_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/crobot_hardware
)
install(TARGETS crobot_hardware
  EXPORT export_crobot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_crobot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${INCLUDE_DEPENDS})
ament_package()
