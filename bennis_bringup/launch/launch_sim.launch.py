import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'bennis_bringup'  # <--- CHANGE ME

    world = LaunchConfiguration('world')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'gazebo_ignition': 'true'}.items()
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.sdf'
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            '-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    gz_spawn_entity_node = Node(package='ros_gz_sim', executable='create',
                                arguments=['-topic', 'robot_description',
                                           '-name', 'bennis',
                                           '-z', '0.1'],
                                output='screen')

    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    bridge_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='World to load'
        )
    )
    ld.add_action(rsp)
    ld.add_action(gz_sim_node)
    ld.add_action(gz_spawn_entity_node)
    ld.add_action(ros_gz_bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(mecanum_drive_spawner)

    return ld


# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true
