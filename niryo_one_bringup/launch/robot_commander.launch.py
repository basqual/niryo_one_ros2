from inspect import Parameter
from operator import truediv
import os
from pickle import TRUE
import xacro

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression,Command,PathJoinSubstitution,FindExecutable,ThisLaunchFileDir
from launch_ros.actions import Node



def generate_launch_description():
    moveit_config_dir = get_package_share_directory('niryo_one_moveit_config')

    description_dir_argument = DeclareLaunchArgument(name="description_dir")
    urdf_file_argument = DeclareLaunchArgument(name="urdf_file")
    namespace_argument = DeclareLaunchArgument(name="ns")
    launch_rviz_argument = DeclareLaunchArgument(name="launch_rviz")

    description_dir = LaunchConfiguration('description_dir')
    urdf_file = LaunchConfiguration('urdf_file')
    namespace = LaunchConfiguration("ns")
    launch_rviz = LaunchConfiguration("launch_rviz")

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_config_dir,"/launch" ,"/niryo_one_moveit.launch.py"]),
        launch_arguments={
            "description_dir":description_dir,
            "urdf_file":urdf_file,
            "moveit_config_package":"niryo_one_moveit_config",
            "moveit_config_file":"niryo_one.srdf.xacro",
            "ns":namespace,
            "use_sim_time": "true",
            "launch_rviz":launch_rviz
        }.items(),
    )
        

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(description_dir_argument)
    ld.add_action(urdf_file_argument)
    ld.add_action(namespace_argument)
    ld.add_action(launch_rviz_argument)
    ld.add_action(move_group_launch)

    return ld   