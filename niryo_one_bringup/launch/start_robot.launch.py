from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,ThisLaunchFileDir,TextSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node



def generate_launch_description():
    description_dir = get_package_share_directory('niryo_one_description')
    urdf_file= 'niryo_one.urdf.xacro'

    description_dir_argument = DeclareLaunchArgument(name="description_dir",default_value = TextSubstitution(text=description_dir))
    urdf_file_argument = DeclareLaunchArgument(name="urdf_file",default_value = TextSubstitution(text=urdf_file))
    namespace_argument = DeclareLaunchArgument("ns", default_value=TextSubstitution(text=""))
    hardware_version_launch_arg = DeclareLaunchArgument("hardware_version", default_value=TextSubstitution(text="2"))

    launch_controllers_with_namespace = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(LaunchConfiguration('ns')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/controllers.launch.py"]),
                launch_arguments={
                    "fake_communication": "false",
                    "ns":LaunchConfiguration("ns"), 
                    "description_dir":LaunchConfiguration("description_dir"),
                    "urdf_file":LaunchConfiguration("urdf_file"),
                    "hardware_version":LaunchConfiguration('hardware_version')
        }.items(),
        ),
        ]
    )

    start_niryo_one_rpi_cmd = Node(
        package='niryo_one_rpi',
        executable='niryo_one_rpi',
        name='niryo_one_rpi',
        respawn='false',
        output='screen',
        namespace=LaunchConfiguration('ns')
    )

  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(namespace_argument)
    ld.add_action(hardware_version_launch_arg)
    ld.add_action(description_dir_argument)
    ld.add_action(urdf_file_argument)
    ld.add_action(start_niryo_one_rpi_cmd)
    ld.add_action(launch_controllers_with_namespace)
    return ld   