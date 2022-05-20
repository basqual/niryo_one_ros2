cmake_minimum_required(VERSION 3.5)
project(mcp_can_rpi)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fPIC)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


#ament_export_include_directories(include)
ament_export_libraries(mcp_can_rpi)

ament_export_targets(mcp_can_rpi HAS_LIBRARY_TARGET)



###########
## Build ##
###########

include_directories(
  include
)
set(HEADER_FILES 
  include/mcp_can_rpi/mcp_can_rpi.h
)

add_library(mcp_can_rpi
    src/mcp_can_rpi.cpp
    ${HEADER_FILES}
)

ament_target_dependencies(mcp_can_rpi
  rclcpp
)

#add_dependencies(mcp_can_rpi ${PROJECT_NAME}_EXPORTED_TARGETS)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "aarch64")
    message(STATUS "wiringPi library is required for mcp_can_rpi (aarch64 processor)")
    target_link_libraries(mcp_can_rpi
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(mcp_can_rpi 
        ${catkin_LIBRARIES} 
    )
endif()

#############
## Install ##
#############
install(TARGETS mcp_can_rpi
  EXPORT mcp_can_rpi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_package()