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

    parameters = [params_file]

    # Empty use_sim_time means "defer to the profile yaml" (so profile:=sim keeps
    # its use_sim_time: true); only override when the arg was actually given.
    if LaunchConfiguration("use_sim_time").perform(context) != "":
        parameters.append(
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }
        )

    parameters.append(
        {
            "autostart": ParameterValue(
                LaunchConfiguration("autostart"), value_type=bool
            )
        }
    )

    core = Node(
        package="rov2_core",
        executable="rov2_core_node",
        name="rov2_core",
        output="screen",
        parameters=parameters,
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
                default_value="",
                description=(
                    "Consume /clock from a simulator (e.g. remote Isaac Sim). "
                    "Empty defers to the profile yaml; profile:=sim already sets true."
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Auto-drive the lifecycle node to the active state on start",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
