# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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

import os
from distutils.util import strtobool

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    simulation = bool(strtobool(os.environ.get('SIMULATION')))

    arg_use_sensor_fusion = LaunchConfiguration('use_sensor_fusion')

    return LaunchDescription([
        DeclareLaunchArgument('use_sensor_fusion', default_value='False'),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml'),
                {'use_sim_time': simulation}],
            condition=IfCondition(arg_use_sensor_fusion),
           ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='odom_transform_node.py',
            name='odom_transform_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml'),
                {'use_sim_time': simulation}],
            condition=IfCondition(arg_use_sensor_fusion),
           ),
        launch_ros.actions.Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("ros2_laser_scan_matcher"), 'params', 'matcher.yaml'),
                {'use_sim_time': simulation}],
           ),
])
