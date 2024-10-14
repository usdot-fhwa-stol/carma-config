# Copyright (C) 2021-2024 LEIDOS.
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
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from carma_ros2_utils.launch.generate_log_levels import generate_log_levels
import os

def generate_launch_description():
    """
    Launch CARMA System.
    """
    
    # Parse the log config file and convert it to an environment variable
    config_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'carma_rosconsole.conf') 
    logging_env_var = SetEnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', generate_log_levels(config_file_path))

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

    # Declare launch arguments for points_map_loader
    load_type = LaunchConfiguration('load_type')
    declare_load_type= DeclareLaunchArgument(name = 'load_type', default_value = "noupdate")

    single_pcd_path = LaunchConfiguration('single_pcd_path')
    declare_single_pcd_path = DeclareLaunchArgument(name='single_pcd_path', default_value="['/opt/carma/maps/pcd_map.pcd']")

    area = LaunchConfiguration('area')
    declare_area = DeclareLaunchArgument(name='area', default_value="1x1")

    arealist_path = LaunchConfiguration('arealist_path')
    declare_arealist_path = DeclareLaunchArgument(name='arealist_path', default_value="/opt/carma/maps/arealist.txt")

    vector_map_file = LaunchConfiguration('vector_map_file')
    declare_vector_map_file = DeclareLaunchArgument(name='vector_map_file', default_value='/opt/carma/maps/vector_map.osm')

    #Declare the route file folder launch argument
    route_file_folder = LaunchConfiguration('route_file_folder')
    declare_route_file_folder = DeclareLaunchArgument(
        name = 'route_file_folder',
        default_value='/opt/carma/routes/',
        description = 'Path of folder containing routes to load'
    )

    # Declare enable_guidance_plugin_validate
    enable_guidance_plugin_validator = LaunchConfiguration('enable_guidance_plugin_validator')
    declare_enable_guidance_plugin_validator = DeclareLaunchArgument(
        name = 'enable_guidance_plugin_validator', 
        default_value='true', 
        description='Flag indicating whether the Guidance Plugin Validator node will actively validate guidance strategic, tactical, and control plugins'
    )

    # Declare strategic_plugins_to_validate
    strategic_plugins_to_validate = LaunchConfiguration('strategic_plugins_to_validate')
    declare_strategic_plugins_to_validate = DeclareLaunchArgument(
        name = 'strategic_plugins_to_validate',
        default_value = '[RouteFollowing]',
        description = 'List of String: Guidance Strategic Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare tactical_plugins_to_validate
    tactical_plugins_to_validate = LaunchConfiguration('tactical_plugins_to_validate')
    declare_tactical_plugins_to_validate = DeclareLaunchArgument(
        name = 'tactical_plugins_to_validate',
        default_value='[InLaneCruisingPlugin, StopandWaitPlugin, CooperativeLaneChangePlugin, UnobstructedLaneChangePlugin, YieldPlugin]',
        description='List of String: Guidance Tactical Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare control_plugins_to_validate
    control_plugins_to_validate = LaunchConfiguration('control_plugins_to_validate')
    declare_control_plugins_to_validate = DeclareLaunchArgument(
        name = 'control_plugins_to_validate',
        default_value= '[Pure Pursuit]',
        description='List of String: Guidance Control Plugins that will be validated by the Guidance Plugin Validator Node if enabled'
    )

    # Declare enable_opening_tunnels
    enable_opening_tunnels = LaunchConfiguration('enable_opening_tunnels')
    declare_enable_opening_tunnels = DeclareLaunchArgument(
        name = 'enable_opening_tunnels',
        default_value= 'False',
        description='Flag to enable opening http tunnesl to CARMA Cloud'
    )

    # Declare is_ros2_tracing_enabled
    is_ros2_tracing_enabled = LaunchConfiguration('is_ros2_tracing_enabled')
    declare_is_ros2_tracing_enabled = DeclareLaunchArgument(
        name='is_ros2_tracing_enabled', 
        default_value = 'False', 
        description = 'True if user wants ROS 2 Tracing logs to be generated from CARMA Platform'
    )

    # Launch the core carma launch file
    carma_src_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ get_package_share_directory('carma'), '/launch/carma_src.launch.py']),
        launch_arguments = {
            'vehicle_calibration_dir' : vehicle_calibration_dir,
            'vehicle_config_dir' : vehicle_config_dir,
            'route_file_folder' : route_file_folder,
            'enable_guidance_plugin_validator' : enable_guidance_plugin_validator,
            'strategic_plugins_to_validate' : strategic_plugins_to_validate,
            'tactical_plugins_to_validate' : tactical_plugins_to_validate,
            'control_plugins_to_validate' : control_plugins_to_validate,
            'load_type' : load_type,
            'single_pcd_path' : single_pcd_path,
            'area' : area,
            'arealist_path' : arealist_path,
            'vector_map_file' : vector_map_file
            }.items()
    )

    return LaunchDescription([
        logging_env_var, # Environment variables must be placed before included files
        declare_vehicle_calibration_dir_arg,
        declare_vehicle_config_dir_arg,
        declare_route_file_folder,
        declare_enable_guidance_plugin_validator,
        declare_strategic_plugins_to_validate,
        declare_tactical_plugins_to_validate,
        declare_control_plugins_to_validate,
        declare_enable_opening_tunnels,
        declare_load_type,
        declare_single_pcd_path,
        declare_area,
        declare_arealist_path,
        declare_vector_map_file,
        declare_is_ros2_tracing_enabled,
        carma_src_launch
    ])
