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
<<<<<<< HEAD
from ament_index_python.packages import get_package_share_directory
=======
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

>>>>>>> ef7154e... Sensor fusion (#5)
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description() -> LaunchDescription:
    simulation = bool(strtobool(os.environ.get('SIMULATION')))

    arg_sensor_fusion = LaunchConfiguration('sensor_fusion')

    robot_loc_params = os.path.join(
        get_package_share_directory('robot_localization'),
        'config', 'ekf.yaml')

    matcher_dir = get_package_share_directory('ros2_laser_scan_matcher')

    return LaunchDescription([
        DeclareLaunchArgument('sensor_fusion', default_value='True'),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_loc_params, {'use_sim_time': simulation}],
            condition=IfCondition(arg_sensor_fusion),
        ),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='odom_transform_node.py',
            name='odom_transform_node',
            output='screen',
            parameters=[robot_loc_params, {'use_sim_time': simulation}],
            condition=IfCondition(arg_sensor_fusion),
        ),

        launch_ros.actions.Node(
            package='ros2_laser_scan_matcher',
            executable='laser_scan_matcher',
            name='laser_scan_matcher',
            output='screen',
            parameters=[os.path.join(matcher_dir, 'config', 'matcher.yaml'),
                        {'use_sim_time': simulation, 'publish_tf': False}],
            remappings=[
                ('pose_with_covariance_stamped', 'laser_pose')],
        ),
    ])
