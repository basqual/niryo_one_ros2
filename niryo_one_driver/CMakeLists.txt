cmake_minimum_required(VERSION 3.5)
project(niryo_one_driver)

## Compile as C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 14)


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(niryo_one_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mcp_can_rpi REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)



set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp 
  rclcpp_action
  std_msgs 
  hardware_interface 
  controller_manager 
  control_msgs 
  geometry_msgs 
  sensor_msgs 
  trajectory_msgs
  niryo_one_msgs
  dynamixel_sdk
  mcp_can_rpi
  pluginlib
  urdf
)

include_directories(include)

add_library(niryo_one_hardware_plugin
    SHARED
    src/niryo_one_hardware_interface.cpp
    src/test_motors.cpp
    src/ros_interface.cpp
    src/rpi_diagnostics.cpp
    src/hw_driver/niryo_one_can_driver.cpp
    src/hw_driver/dxl_driver.cpp
    src/hw_driver/xl320_driver.cpp
    src/hw_driver/xl430_driver.cpp
    src/hw_comm/dxl_communication.cpp
    src/hw_comm/can_communication.cpp
    src/hw_comm/niryo_one_communication.cpp
    src/hw_comm/fake_communication.cpp
    src/utils/motor_offset_file_handler.cpp 
)

target_include_directories(
  niryo_one_hardware_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  niryo_one_hardware_plugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "aarch64")
    message(STATUS "wiringPi library is required for niryo_one_hardware_plugin (aarch64 processor)")
    target_link_libraries(niryo_one_hardware_plugin
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(niryo_one_hardware_plugin 
        ${catkin_LIBRARIES} 
    )
endif()

# INSTALL
install(
  TARGETS niryo_one_hardware_plugin 
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  niryo_one_hardware_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()

