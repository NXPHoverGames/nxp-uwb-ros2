#!/usr/bin/python3
# Copyright 2025 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('ranging_interval', default_value=['50'],
                          description='Ranging interval in ms'),
    DeclareLaunchArgument('position', description='Relative position of anchor x,y,z in meters'),
    DeclareLaunchArgument('controller_mac', default_value='[0x1111]',
                          description='Mac address of the anchor controller'),
    DeclareLaunchArgument('anchor_mac', default_value='[0x3333]',
                          description='Mac address of the anchor responder'),
]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [
        Node(
            package='nxp-uwb-ros2',
            namespace='sr1xx',
            executable='node',
            name='tdoa_dl_anchor_controlee',
            parameters=[
                {'role': 'tdoa_dl_anchor_controlee'},
                {'device_mac_address': LaunchConfiguration('anchor_mac')},
                {'dst_mac_address': LaunchConfiguration('controller_mac')},
                {'ranging_interval': LaunchConfiguration('ranging_interval')},
                {'position': LaunchConfiguration('position')}
            ],
        ),
    ])
