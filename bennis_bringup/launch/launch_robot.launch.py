import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='bennis_bringup' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('bennis_bringup'),'launch','rsp.launch.py'
                )]), launch_arguments={'gazebo_ignition': 'false'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )


    twist_mux_params = os.path.join(get_package_share_directory('bennis_description'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory('bennis_description'),'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
        remappings=[
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
            ('/mecanum_drive_controller/odometry', '/odom'),
            ('/mecanum_drive_controller/reference', '/cmd_vel'),
        ]
    )

    delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager])

    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['mecanum_drive_controller'],
    )

    delayed_mecanum_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[mecanum_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    ld = LaunchDescription()

    ld.add_action(rsp)
    # ld.add_action(twist_mux)
    ld.add_action(delayed_controller_manager)
    ld.add_action(delayed_mecanum_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)

    return ld   

    