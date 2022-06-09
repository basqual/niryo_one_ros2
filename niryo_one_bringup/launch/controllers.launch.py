from inspect import Parameter
from operator import truediv
import os
from pickle import TRUE

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,SetEnvironmentVariable,LogInfo
from launch.conditions import IfCondition,LaunchConfigurationEquals,LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression,Command,PathJoinSubstitution,FindExecutable,EnvironmentVariable,TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
import yaml




def generate_launch_description():
    bringup_dir = get_package_share_directory('niryo_one_bringup')
    #
    namespace_argument = DeclareLaunchArgument(name="ns")
    description_dir_argument = DeclareLaunchArgument(name="description_dir")
    urdf_file_argument = DeclareLaunchArgument(name="urdf_file")
    simulation_mode_argument = DeclareLaunchArgument(name="fake_communication")
    hardware_version_launch_arg = DeclareLaunchArgument("hardware_version")
    version_folder_launch_arg = DeclareLaunchArgument('version_folder', default_value=['v',LaunchConfiguration('hardware_version')])

    description_dir = LaunchConfiguration('description_dir')
    urdf_file = LaunchConfiguration('urdf_file')
    namespace = LaunchConfiguration("ns")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_dir, "urdf", urdf_file]),
            " ",
            "prefix:=",
            namespace,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config = PathJoinSubstitution([
        bringup_dir,
        'config',
        'niryo_one_driver.yaml'
        ])
    toolConfig = PathJoinSubstitution([
        get_package_share_directory('niryo_one_tools'),
        'config',
        'end_effectors.yaml'
        ])
    controllers_config = PathJoinSubstitution([
        get_package_share_directory('niryo_one_driver'),
        'config',
        'niryo_one_controllers.yaml'
        ])
    motors_config = PathJoinSubstitution([
        get_package_share_directory('niryo_one_bringup'),
        'config',
        LaunchConfiguration('version_folder'),
        'niryo_one_motors.yaml'
        ])
    steppers_config = PathJoinSubstitution([
        get_package_share_directory('niryo_one_bringup'),
        'config',
        LaunchConfiguration('version_folder'),
        'stepper_params.yaml'
        ])

    

    start_niryo_one_driver_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
                robot_description,
                {"fake_communication":LaunchConfiguration('fake_communication')},
                {"hardware_version":LaunchConfiguration('hardware_version')},
                ParameterFile(motors_config,allow_substs=True),
                ParameterFile(steppers_config,allow_substs=True),
                ParameterFile(config,allow_substs=True),
                ParameterFile(controllers_config, allow_substs=True),
            ],
        condition = LaunchConfigurationEquals('ns','')
    )

    start_niryo_one_driver_cmd2 = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
                robot_description,
                {"fake_communication":LaunchConfiguration('fake_communication')},
                {"hardware_version":LaunchConfiguration('hardware_version')},
                ParameterFile(motors_config,allow_substs=True),
                ParameterFile(steppers_config,allow_substs=True),
                ParameterFile(config,allow_substs=True),
                ParameterFile(controllers_config, allow_substs=True),
                
            ],
        remappings=[
                (['/',LaunchConfiguration('ns'),'/joint_states'], '/joint_states'),
            ],
        condition = LaunchConfigurationNotEquals('ns','')
    )


    start_niryo_one_tools_cmd = Node(
        package='niryo_one_tools',
        executable='niryo_one_tools',
        name='niryo_one_tools',
        respawn=False,
        output='screen',
        parameters=[ParameterFile(toolConfig, allow_substs=True)]
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    start_controller_spawner_1_cmd = Node(
        package='controller_manager',
        output='screen',
        executable='spawner',
        arguments= ['niryo_one_follow_joint_trajectory_controller','-c',[namespace,"/controller_manager"]]
    )

    start_controller_spawner_2_cmd = Node(
        package='controller_manager',
        output='screen',
        executable='spawner',
        arguments= ['gripper_controller','-c',[namespace,"/controller_manager"]]
    )

    start_joint_state_broadcaster_cmd = Node(
        package='controller_manager',
        output='screen',
        executable='spawner',
        arguments=['joint_state_broadcaster','-c',[namespace,"/controller_manager"]]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(version_folder_launch_arg)
    ld.add_action(start_niryo_one_driver_cmd2)
    ld.add_action(simulation_mode_argument)
    ld.add_action(namespace_argument)
    ld.add_action(description_dir_argument)
    ld.add_action(urdf_file_argument)
    ld.add_action(hardware_version_launch_arg)
    ld.add_action(start_niryo_one_driver_cmd)
    #ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_niryo_one_tools_cmd)
    ld.add_action(start_controller_spawner_1_cmd)
    ld.add_action(start_controller_spawner_2_cmd)
    ld.add_action(start_joint_state_broadcaster_cmd)
    

    return ld   