"""Bring up the ROV2 core runtime with a selectable profile.

profile selects the parameter file under config/ (dev | sim | hardware).
use_sim_time and autostart can override the profile file at launch time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    params_file = os.path.join(
        get_package_share_directory("rov2_bringup"), "config", f"{profile}.yaml"
    )

    core = Node(
        package="rov2_core",
        executable="rov2_core_node",
        name="rov2_core",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool
                ),
            },
        ],
    )
    return [core]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="dev",
                description="Bringup profile: dev | sim | hardware",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Consume /clock from a simulator (e.g. remote Isaac Sim)",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Auto-drive the lifecycle node to the active state on start",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
