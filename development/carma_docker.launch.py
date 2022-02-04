# Copyright (C) 2021-2022 LEIDOS.
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

    # Launch the core carma launch file
    carma_src_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ get_package_share_directory('carma'), '/launch/carma_src.launch.py']),
        launch_arguments = {
            'vehicle_calibration_dir' : vehicle_calibration_dir,
            'vehicle_config_dir' : vehicle_config_dir
            }.items()
    )

    return LaunchDescription([
        logging_env_var, # Environment variables must be placed before included files
        declare_vehicle_calibration_dir_arg,
        declare_vehicle_config_dir_arg,
        carma_src_launch
    ])
