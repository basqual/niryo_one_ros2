################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

set(DYNAMIXEL_SDK_SOURCES
  src/packet_handler.cpp
  src/protocol1_packet_handler.cpp
  src/protocol2_packet_handler.cpp
  src/group_sync_read.cpp
  src/group_sync_write.cpp
  src/group_bulk_read.cpp
  src/group_bulk_write.cpp
  src/port_handler.cpp
)

if(APPLE)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/port_handler_mac.cpp
  )
elseif(WIN32)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/port_handler_windows.cpp
  )
else()
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/port_handler_linux.cpp
  )
endif()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "aarch64")
    message(STATUS "wiringPi library is required for dynamixel_sdk (aarch64 processor)")
    target_link_libraries(dynamixel_sdk
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(dynamixel_sdk 
        ${catkin_LIBRARIES} 
    )
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()