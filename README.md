# Niryo One Ros2 
**IMPORTANT: This is not an official Niryo One ROS2 port!**

However, this provides a fully functional ROS2 Stack for the Niryo One, although only the core functions have been ported. Feel free to contribute more.

This port added a few features, including:
- Full namespacing support
- Gripper controller (At the moment, command_interface is disabled)

## Install
Clone the repository and build with ```colcon build```

## Usage
Start the driver with ```ros2 launch niryo_one_bringup start_sim.launch.py``` for simulation or ```ros2 launch niryo_one_bringup start_robot.launch.py``` for the actual robot. Namespace the driver by adding ```ns:=foo``` after the last argument 


Currently, the User interface has not been ported, so the calibration and learning mode have to be called manually with the ROS2 service CLI.
- ```ros2 service call /niryo_one/calibrate_motors niryo_one_msgs/srv/SetInt "{value : 1}"```
- ```ros2 service call /niryo_one/activate_learning_mode niryo_one_msgs/srv/SetInt "{value : 0}"``` (0 = disable)

## Known Issues
- niryo_one_tools services not yet fully functional
- Gripper joint_state reading is off
