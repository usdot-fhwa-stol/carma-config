# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.substitutions import LaunchConfiguration

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap

def generate_launch_description():
    """
    Launch driver subsystem nodes.
    This file is used for vehicle configurations. The arguments from carma.launch should pass on to carma_src.launch, which should then pass those arguments to this file. The arguments in the carma.launch file will override all of the default values of the arguments being passed, so you should be making changes to the carma.launch to configure it to your vehicle. 

    If not using simulated drivers they are activated if the respective mock arguments being passed in are false. These lines below activate the respective actual driver if the respective mock argument being passed is false.
    """

    vehicle_ssc_param_dir = LaunchConfiguration('vehicle_ssc_param_dir')

    # Specific Drivers
    # ssc_interface_wrapper
    # Pacmod driver
    # pacmod_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/drivers.launch.py']),
    # )

    return LaunchDescription([        

    ])