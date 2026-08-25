"""One-command Milestone 2 sim bringup.

Starts the framework core in the sim profile (which already enables use_sim_time)
and the sim_pilot velocity source, so a single command drives both the framework
(MockActuator) and the remote Isaac Sim rover over the shared cmd_vel topic.

Observe state coming back with, e.g.:
  ros2 topic echo /rov2_core/status        # MockActuator reports lin.x/ang.z
  ros2 topic echo /odom                     # from the sim
  ros2 run tf2_ros tf2_echo odom base_link  # from the sim
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rov2_launch = os.path.join(
        get_package_share_directory("rov2_bringup"), "launch", "rov2.launch.py"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_pilot = LaunchConfiguration("start_pilot")

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rov2_launch),
        launch_arguments={"profile": "sim"}.items(),
    )

    pilot = Node(
        package="rov2_sim",
        executable="sim_pilot",
        name="sim_pilot",
        output="screen",
        condition=IfCondition(start_pilot),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "linear_speed": ParameterValue(
                    LaunchConfiguration("linear_speed"), value_type=float
                ),
                "angular_speed": ParameterValue(
                    LaunchConfiguration("angular_speed"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Consume /clock from the remote Isaac Sim (recommended for M2)",
            ),
            DeclareLaunchArgument(
                "start_pilot",
                default_value="true",
                description="Start the sim_pilot velocity source alongside the core",
            ),
            DeclareLaunchArgument(
                "linear_speed",
                default_value="0.3",
                description="sim_pilot forward speed (m/s)",
            ),
            DeclareLaunchArgument(
                "angular_speed",
                default_value="0.5",
                description="sim_pilot yaw rate during the arc phase (rad/s)",
            ),
            core,
            pilot,
        ]
    )
