# Copyright (C) 2022-2026 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch the selected ROS 2 vehicle drivers."""

import uuid

from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    env_log_levels = EnvironmentVariable(
        "CARMA_ROS_LOGGING_CONFIG",
        default_value='{ "default_level" : "WARN" }',
    )
    drivers = LaunchConfiguration("drivers")

    driver_shutdown_group = GroupAction(
        actions=[
            PushRosNamespace(
                EnvironmentVariable(
                    "CARMA_INTR_NS", default_value="hardware_interface"
                )
            ),
            Node(
                package="driver_shutdown_ros2",
                executable="driver_shutdown_ros2_node_exec",
                name=["driver_shutdown_", uuid.uuid4().hex],
                on_exit=Shutdown(),
                arguments=[
                    "--ros-args",
                    "--log-level",
                    GetLogLevel("driver_shutdown_ros2", env_log_levels),
                ],
            ),
        ]
    )

    v2x_driver_group = GroupAction(
        condition=IfCondition(
            PythonExpression(
                ["'v2x_ros_driver' in '", drivers, "'.split()"]
            )
        ),
        actions=[
            PushRosNamespace(
                EnvironmentVariable(
                    "CARMA_INTR_NS", default_value="hardware_interface"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("v2x_ros_driver"),
                        "/launch/v2x_ros_driver.launch.py",
                    ]
                ),
                launch_arguments={
                    "log_level": GetLogLevel(
                        "v2x_ros_driver", env_log_levels
                    )
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drivers",
                default_value="v2x_ros_driver",
                description="Space-separated driver package names to launch",
            ),
            driver_shutdown_group,
            v2x_driver_group,
        ]
    )
