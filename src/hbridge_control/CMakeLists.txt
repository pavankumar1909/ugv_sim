cmake_minimum_required(VERSION 3.8)
project(hbridge_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)


include_directories(include/motor_interface)

#add_executable(hbridge_control src/ros2_hbridge.cpp)

add_library(${PROJECT_NAME} SHARED src/mobile_base_hardware_interface.cpp)

# Link dependencies correctly
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface hbridge_control_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin XML
install(
  DIRECTORY include/motor_interface include
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES hbridge_control_interface.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_export_libraries(
  ${PROJECT_NAME}
)


ament_export_dependencies(
   rclcpp_lifecycle
   pluginlib
   rclcpp
   hardware_interface
)


ament_package()
