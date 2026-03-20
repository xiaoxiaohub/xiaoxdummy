cmake_minimum_required(VERSION 3.8)
project(xiaoxdummy_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(xiaoxdummy_interface REQUIRED)

add_library(xiaoxdummy_hardware SHARED src/xiaoxdummy_hardware.cpp)
ament_target_dependencies(
  xiaoxdummy_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  xiaoxdummy_interface
)

pluginlib_export_plugin_description_file(hardware_interface xiaoxdummy_hardware.xml)

install(TARGETS xiaoxdummy_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
