cmake_minimum_required(VERSION 3.5)
project(actuator_state_broadcaster)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(actuator_state_broadcaster_parameters
  src/actuator_state_broadcaster_parameters.yaml
)

add_library(actuator_state_broadcaster
  SHARED
  src/actuator_state_broadcaster.cpp
)
target_include_directories(actuator_state_broadcaster PRIVATE include)
ament_target_dependencies(actuator_state_broadcaster
  builtin_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  sensor_msgs
)
target_link_libraries(actuator_state_broadcaster
actuator_state_broadcaster_parameters
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(actuator_state_broadcaster PRIVATE "ACTUATOR_STATE_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface actuator_state_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  actuator_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  control_msgs
  controller_interface
  generate_parameter_library
  rclcpp_lifecycle
  sensor_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  actuator_state_broadcaster
)
ament_package()
