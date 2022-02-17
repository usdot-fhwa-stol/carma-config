# Copyright (C) 2022 LEIDOS.
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
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel

def generate_launch_description():
    """
    Launch CARMA System.
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    # Declare the vehicle_calibration_dir launch argument
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    declare_vehicle_calibration_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_calibration_dir', default_value = '/opt/carma/vehicle/calibration', description = "Path to vehicle calibration directory"
    )

    # Declare the vehicle_config_dir launch argument
    vehicle_config_dir = LaunchConfiguration('vehicle_config_dir')
    declare_vehicle_config_dir_arg = DeclareLaunchArgument(
        name = 'vehicle_config_dir', default_value = '/opt/carma/vehicle/config', description = "Path to vehicle configuration directory"
    )

    drivers = LaunchConfiguration('drivers')
    declare_drivers_arg = DeclareLaunchArgument(
        name = 'drivers', default_value = 'dsrc_driver', description = "Desired drivers to launch specified by package name."
    )
    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(
        name = 'ssc_package_name', default_value = 'ssc_pm_lexus', description = "Third party ssc package to launch" 
    )

    dsrc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'dsrc_driver' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ get_package_share_directory('dsrc_driver'), '/launch/dsrc_driver.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('dsrc_driver', env_log_levels),
                    }.items()
            ),
        ]
    )

    ssc_group = GroupAction(
        # Launch ssc
        condition=IfCondition(PythonExpression(["'ssc_interface_wrapper_ros2' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ get_package_share_directory('ssc_interface_wrapper_ros2'), '/launch/ssc_pacmod_driver.launch.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('ssc_interface_wrapper_ros2', env_log_levels),
                    'vehicle_calibration_dir' : vehicle_calibration_dir,
                    'ssc_package_name' : ssc_package_name
                    }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_drivers_arg,
        declare_vehicle_calibration_dir_arg,
        declare_vehicle_config_dir_arg,
        declare_ssc_package_name,
        dsrc_group,
        ssc_group
    ])