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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    bennis_model_config = LaunchConfiguration('bennis_model')
    serial_port = LaunchConfiguration('serial_port')
    prefix = LaunchConfiguration('prefix')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("bennis_description"), "urdf", "bennis.urdf.xacro"]),
        " ",
        "bennis_model:=",
        bennis_model_config,
        " ",
        "serial_port:=",
        serial_port,
        " ",
        "prefix:=",
        prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bennis_description"), "rviz", "rviz_robot.rviz"]
    )


    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port to connect to the robot",
        ))
    ld.add_action(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for Bennis tf_tree",
        ))
    ld.add_action(
        DeclareLaunchArgument("bennis_model",
                              default_value="bennis",
                              choices=["bennis"],
                              description="Model of Bennis"))

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld
