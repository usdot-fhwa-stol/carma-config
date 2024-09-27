# Copyright (C) 2024 LEIDOS.
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch.actions import Shutdown
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from carma_ros2_utils.launch.get_log_level import GetLogLevel
import uuid

def generate_launch_description():
    """
    Launch CARMA System.
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    drivers = LaunchConfiguration('drivers')
    declare_drivers_arg = DeclareLaunchArgument(
        name = 'drivers', default_value = 'mock_controller_driver', description = "Desired mock drivers to launch specified by package name."
    )

    # Launch shutdown node which will ensure the launch file gets closed on system shutdown even if in a separate container
    driver_shutdown_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            Node (
                package='driver_shutdown_ros2',
                executable='driver_shutdown_ros2_node_exec',
                name=[ 'driver_shutdown_', uuid.uuid4().hex ], # No clear way to make anonymous nodes in ros2, so for now we will generate a uuid for now
                on_exit=Shutdown(),
                arguments=['--ros-args', '--log-level', GetLogLevel('driver_shutdown_ros2', env_log_levels)]
            )
        ]
    )

    mock_controller_group = GroupAction(
        condition=IfCondition(PythonExpression(["'mock_controller_driver' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ FindPackageShare('mock_controller_driver'), '/launch/mock_controller_driver.launch.py']),
                launch_arguments = {
                    'log_level' : GetLogLevel('mock_controller_driver', env_log_levels),
                    }.items()
            ),
        ]
    )


    return LaunchDescription([
        declare_drivers_arg,
        driver_shutdown_group,
        mock_controller_group
    ])
