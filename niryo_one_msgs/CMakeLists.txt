cmake_minimum_required(VERSION 3.5)
project(niryo_one_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED COMPONENTS
  rclcpp 
  rospy
  message_generation
)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JoystickJoints.action"
  "action/RobotMove.action"
  "action/Tool.action"
  "action/Sequence.action"
  "msg/RPY.msg"
  "msg/ShiftPose.msg"
  "msg/RobotState.msg"
  "msg/RobotMoveCommand.msg"
  "msg/ToolCommand.msg"
  "msg/ProcessState.msg"
  "msg/HardwareStatus.msg"
  "msg/LogStatus.msg"
  "msg/DigitalIOState.msg"
  "msg/SoftwareVersion.msg"
  "msg/Sequence.msg"
  "msg/SequenceAutorunStatus.msg"
  "msg/TrajectoryPlan.msg"
  "msg/MatlabMoveResult.msg"
  "msg/Position.msg"
  "msg/Trajectory.msg"
  "msg/ObjectPose.msg"
  "msg/ConveyorFeedback.msg"
  "srv/ChangeHardwareVersion.srv"
  "srv/ChangeMotorConfig.srv"
  "srv/ControlConveyor.srv"
  "srv/CloseGripper.srv"
  "srv/DebugColorDetection.srv"
  "srv/DebugMarkers.srv"
  "srv/EditGrip.srv"
  "srv/EditWorkspace.srv"
  "srv/GetCalibrationCam.srv"
  "srv/GetDigitalIO.srv"
  "srv/GetInt.srv"
  "srv/GetPositionList.srv"
  "srv/GetSequenceList.srv"
  "srv/GetTargetPose.srv"
  "srv/GetTrajectoryList.srv"
  "srv/GetWorkspaceList.srv"
  "srv/GetWorkspaceRatio.srv"
  "srv/GetWorkspaceRobotPoses.srv"
  "srv/ObjDetection.srv"
  "srv/ManagePosition.srv"
  "srv/ManageProcess.srv"
  "srv/ManageSequence.srv"
  "srv/ManageTrajectory.srv"
  "srv/OpenGripper.srv"
  "srv/PingDxlTool.srv"
  "srv/PullAirVacuumPump.srv"
  "srv/PushAirVacuumPump.srv"
  "srv/RobotMove.srv"
  "srv/SendCustomDxlValue.srv"
  "srv/SetBool.srv"
  "srv/SetCalibrationCam.srv"
  "srv/SetConveyor.srv"
  "srv/SetDigitalIO.srv"
  "srv/SetInt.srv"
  "srv/SetLeds.srv"
  "srv/SetSequenceAutorun.srv"
  "srv/SetString.srv"
  "srv/TakePicture.srv"
  "srv/UpdateConveyorId.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
  sensor_msgs
  builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
