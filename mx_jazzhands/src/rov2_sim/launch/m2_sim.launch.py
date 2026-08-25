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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("rov2_bringup")
    rov2_launch = os.path.join(bringup_share, "launch", "rov2.launch.py")
    sim_params = os.path.join(bringup_share, "config", "sim.yaml")

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rov2_launch),
        launch_arguments={"profile": "sim"}.items(),
    )

    # sim.yaml's sim_pilot section is the source of truth; a launch arg overrides
    # it only when explicitly given (empty means "defer to the yaml").
    pilot_params = [
        sim_params,
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        },
    ]
    overrides = (("pattern", str), ("linear_speed", float), ("angular_speed", float))
    for name, caster in overrides:
        if LaunchConfiguration(name).perform(context) != "":
            pilot_params.append(
                {name: ParameterValue(LaunchConfiguration(name), value_type=caster)}
            )

    pilot = Node(
        package="rov2_sim",
        executable="sim_pilot",
        name="sim_pilot",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pilot")),
        parameters=pilot_params,
    )
    return [core, pilot]


def generate_launch_description():
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
                "pattern",
                default_value="",
                description=(
                    "Override sim.yaml sim_pilot pattern: 'forward_arc_stop' or "
                    "'figure8' (empty defers to sim.yaml)"
                ),
            ),
            DeclareLaunchArgument(
                "linear_speed",
                default_value="",
                description="Override sim.yaml sim_pilot forward speed (m/s)",
            ),
            DeclareLaunchArgument(
                "angular_speed",
                default_value="",
                description="Override sim.yaml sim_pilot yaw rate (rad/s)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
