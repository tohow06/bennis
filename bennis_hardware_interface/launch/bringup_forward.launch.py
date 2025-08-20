from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg = get_package_share_directory('bennis_hardware_interface')
    # urdf = PathJoinSubstitution([pkg, 'description', 'arduino_mecanum.xacro'])
    controllers = PathJoinSubstitution([pkg, 'config', 'forward_controllers.yaml'])


    urdf = os.path.join(
        get_package_share_directory('bennis_hardware_interface'),
        'description',
        'robot.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='screen',
    )

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['fl_cmd', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['fr_cmd', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['rl_cmd', '--controller-manager', '/controller_manager']),
        Node(package='controller_manager', executable='spawner',
             arguments=['rr_cmd', '--controller-manager', '/controller_manager']),
    ]

    return LaunchDescription([cm] + spawners)
