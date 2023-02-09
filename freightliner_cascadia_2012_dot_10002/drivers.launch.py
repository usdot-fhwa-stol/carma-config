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
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch.actions import Shutdown
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
from carma_ros2_utils.launch.get_log_level import GetLogLevel
import uuid

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
        name = 'drivers', default_value = 'dsrc_driver velodyne_lidar_driver_wrapper', description = "Desired drivers to launch specified by package name."
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

    dsrc_group = GroupAction(
        condition=IfCondition(PythonExpression(["'dsrc_driver' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ FindPackageShare('dsrc_driver'), '/launch/dsrc_driver.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('dsrc_driver', env_log_levels),
                    }.items()
            ),
        ]
    )

    lidar_group = GroupAction(
        condition=IfCondition(PythonExpression(["'velodyne_lidar_driver_wrapper' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            GroupAction(
                actions=[
                    PushRosNamespace('velodyne_1'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([ FindPackageShare('velodyne_lidar_driver_wrapper'), '/launch/velodyne_lidar_driver_wrapper_launch.py']),
                        launch_arguments = { 
                            'log_level' : GetLogLevel('velodyne_lidar_driver_wrapper', env_log_levels),
                            'frame_id' : 'velodyne_1',
                            'device_ip' : '192.168.1.201',
                            'port' : '2368'
                            }.items()
                    ),
                ]
            ),
            GroupAction(
                actions=[
                    PushRosNamespace('velodyne_2'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([ FindPackageShare('velodyne_lidar_driver_wrapper'), '/launch/velodyne_lidar_driver_wrapper_launch.py']),
                        launch_arguments = { 
                            'log_level' : GetLogLevel('velodyne_lidar_driver_wrapper', env_log_levels),
                            'frame_id' : 'velodyne_2',
                            'device_ip' : '192.168.2.201',
                            'port' : '2369'
                            }.items()
                    ),
                ]
            ),
        ]
    )

    gnss_ins_group = GroupAction(
        condition=IfCondition(PythonExpression(["'carma_novatel_driver_wrapper' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ FindPackageShare('carma_novatel_driver_wrapper'), '/launch/carma-novatel-driver-wrapper-launch.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('carma_novatel_driver_wrapper', env_log_levels),
                    'ip_addr' : '192.168.88.29',
                    'port' : '2000',
                    'vehicle_calibration_dir' : vehicle_calibration_dir,
                    }.items()
            ),
        ]
    )
    
    lightbar_driver_group = GroupAction(
        condition=IfCondition(PythonExpression(["'lightbar_driver' in '", drivers, "'.split()"])),
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_INTR_NS', default_value='hardware_interface')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ FindPackageShare('lightbar_driver'), '/launch/lightbar_driver_node_launch.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('lightbar_driver', env_log_levels),
                    'ip_addr' : '192.168.88.28',
                    'port' : '80',
                    'vehicle_calibration_dir' : vehicle_calibration_dir,
                    }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_drivers_arg,
        declare_vehicle_calibration_dir_arg,
        declare_vehicle_config_dir_arg,
        driver_shutdown_group,
        dsrc_group,
        lidar_group,
        gnss_ins_group,
        lightbar_driver_group
    ])
