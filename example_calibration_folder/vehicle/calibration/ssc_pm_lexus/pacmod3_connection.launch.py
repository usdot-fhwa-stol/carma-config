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

from email.policy import default
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch_ros.actions import set_remap

# This launch file launches the ros2 version pacmod3, used with the ros2 ssc_interface_wrapper
# ros2 should be sourced before using this launch file. The pacmod3 launch file being called here only exists in ros2 version

def generate_launch_description():
    
    pacmod_vehicle_type = LaunchConfiguration('pacmod_vehicle_type')
    declare_pacmod_vehicle_type = DeclareLaunchArgument(name = 'pacmod_vehicle_type', default_value='LEXUS_RX_450H')

    use_kvaser = LaunchConfiguration('use_kvaser')
    declare_use_kvaser = DeclareLaunchArgument(name = 'use_kvaser', default_value='true')

    use_socketcan = LaunchConfiguration('use_socketcan')
    declare_use_socketcan = DeclareLaunchArgument(name = 'use_socketcan', default_value='false')

    socketcan_device = LaunchConfiguration('socketcan_device')
    declare_socketcan_device = DeclareLaunchArgument(name = 'socketcan_device', default_value='can0')

    kvaser_hardware_id = LaunchConfiguration('kvaser_hardware_id')
    declare_kvaser_hardware_id = DeclareLaunchArgument(name='kvaser_hardware_id', default_value='51151')

    kvaser_circuit_id = LaunchConfiguration('kvaser_circuit_id')
    declare_kvaser_circuit_id = DeclareLaunchArgument(name = 'kvaser_circuit_id', default_value = '0')

    kvaser_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_kvaser')),
        actions=[
            set_remap.SetRemap('brake_cmd','as/pacmod/as_rx/brake_cmd'),
            set_remap.SetRemap('brake_rpt', 'as/pacmod/parsed_tx/brake_rpt'),
            set_remap.SetRemap('enabled', 'as/pacmod/as_rx/enable'),
            set_remap.SetRemap('steering_cmd', 'as/pacmod/as_rx/steer_cmd'),
            set_remap.SetRemap('steering_rpt','as/pacmod/parsed_tx/steer_rpt'),
            set_remap.SetRemap('accel_cmd', 'as/pacmod/as_rx/accel_cmd'),
            set_remap.SetRemap('accel_rpt','as/pacmod/parsed_tx/accel_rpt'),
            set_remap.SetRemap('shift_cmd','as/pacmod/as_rx/shift_cmd'),
            set_remap.SetRemap('shift_rpt','as/pacmod/parsed_tx/shift_rpt'),
            set_remap.SetRemap('turn_cmd','as/pacmod/as_rx/turn_cmd'),
            set_remap.SetRemap('vehicle_speed_rpt','as/pacmod/parsed_tx/vehicle_speed_rpt'),
            set_remap.SetRemap('wheel_speed_rpt', 'as/pacmod/parsed_tx/wheel_speed_rpt'),
            set_remap.SetRemap('global_rpt', 'as/pacmod/parsed_tx/global_rpt'),

            ComposableNodeContainer(
                name='kvaser_container',
                namespace= GetCurrentNamespace(),
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[
                    # kvaser_interface
                    ComposableNode(
                        package='kvaser_interface',
                        plugin='kvaser_interface::KvaserReaderNode',
                        name='libkvaser_reader_node',
                        parameters = [
                        {'hardware_id' : kvaser_hardware_id} ,
                        {'circuit_id' : kvaser_circuit_id} ,
                        {'bit_rate' : 500000} #Hardcoded in pacmod launch -  https://github.com/astuff/pacmod3/blob/ros2_master/launch/pacmod3.launch.xml
                        ],
                    ),
                    ComposableNode(
                        package='kvaser_interface',
                        plugin='kvaser_interface::KvaserWriterNode',
                        name='libkvaser_writer_node',
                        parameters = [
                        {'hardware_id' : kvaser_hardware_id} ,
                        {'circuit_id' : kvaser_circuit_id} ,
                        {'bit_rate' : 500000} #Hardcoded in pacmod launch -  https://github.com/astuff/pacmod3/blob/ros2_master/launch/pacmod3.launch.xml
                        ],
                    ),
                    # pacmod3 launch
                    ComposableNode(
                        package='pacmod3',
                        plugin='pacmod3::PACMod3Node',
                        name='pacmod3_node',
                    ),
                ]
            )
        ]
    )
    
    ros2_socketcan_package = FindPackageShare('ros2_socketcan')
    socketcan_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_socketcan')),
        actions=[
            set_remap.SetRemap('brake_cmd','as/pacmod/as_rx/brake_cmd'),
            set_remap.SetRemap('brake_rpt', 'as/pacmod/parsed_tx/brake_rpt'),
            set_remap.SetRemap('enabled', 'as/pacmod/as_rx/enable'),
            set_remap.SetRemap('steering_cmd', 'as/pacmod/as_rx/steer_cmd'),
            set_remap.SetRemap('steering_rpt','as/pacmod/parsed_tx/steer_rpt'),
            set_remap.SetRemap('accel_cmd', 'as/pacmod/as_rx/accel_cmd'),
            set_remap.SetRemap('accel_rpt','as/pacmod/parsed_tx/accel_rpt'),
            set_remap.SetRemap('shift_cmd','as/pacmod/as_rx/shift_cmd'),
            set_remap.SetRemap('shift_rpt','as/pacmod/parsed_tx/shift_rpt'),
            set_remap.SetRemap('turn_cmd','as/pacmod/as_rx/turn_cmd'),
            set_remap.SetRemap('vehicle_speed_rpt','as/pacmod/parsed_tx/vehicle_speed_rpt'),
            set_remap.SetRemap('wheel_speed_rpt', 'as/pacmod/parsed_tx/wheel_speed_rpt'),
            set_remap.SetRemap('global_rpt', 'as/pacmod/parsed_tx/global_rpt'),
            # ros2 socketcan
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource([ros2_socketcan_package, '/launch/socket_can_bridge.launch.xml']),
                launch_arguments ={
                    'socketcan_device' : socketcan_device
                }.items()
            ),
            # Pacmod
            ComposableNodeContainer(
                name='pacmod_container',
                namespace= GetCurrentNamespace(),
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    # pacmod3 launch
                    ComposableNode(
                        package='pacmod3',
                        plugin='pacmod3::PACMod3Node',
                        name='pacmod3_node'
                    ),
                ]
            )
        ]
    )

    return LaunchDescription([
        declare_pacmod_vehicle_type,
        declare_use_kvaser,
        declare_use_socketcan,
        declare_socketcan_device,
        declare_kvaser_hardware_id,
        declare_kvaser_circuit_id,
        kvaser_group,
        socketcan_group
    ])
