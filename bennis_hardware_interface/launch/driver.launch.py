# Copyright 2020 ROS2-Control Development Team (2020)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

import xacro


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('bennis_hardware_interface'),
        'description',
        'robot.urdf')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    joint_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("bennis_hardware_interface"), "config", "controllers.yaml"])

    ld = LaunchDescription()

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(joint_controllers_cfg, allow_substs=True),
        ],
        remappings=[('~/robot_description', 'robot_description')],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    spawn_mecanum_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    
    ld.add_action(controller_manager_node)
    ld.add_action(spawn_joint_controller)

    ld.add_action(robot_state_publisher_node)

    return ld