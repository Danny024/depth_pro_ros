#!/usr/bin/env python

# Copyright (c) 2024 Daniel Eneh <danieleneh024@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    depth_pro_dir = get_package_share_directory('depth_pro_ros')
    default_params_file = os.path.join(depth_pro_dir, 'config', 'params.yaml')
    default_checkpoint_file = os.path.join(depth_pro_dir, 'checkpoints', 'depth_pro.pt')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    checkpoint_file = LaunchConfiguration('checkpoint_file')
    log_level = LaunchConfiguration('log_level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with detection configuration'
    )

    declare_model_file_arg = DeclareLaunchArgument(
        'checkpoint_file',
        default_value=default_checkpoint_file,
        description='Full path to the model file'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'checkpoint_file': checkpoint_file,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Prepare the detection node.
    depth_anything_node = Node(
        package='depth_pro_ros',
        namespace='',
        executable='depth_pro_ros',
        name='depth_pro',
        parameters=[configured_params],
        emulate_tty=True,
        output='screen',
        arguments=[
            '--ros-args',
            '--log-level', ['depth_pro:=', LaunchConfiguration('log_level')]]
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_model_file_arg,
        declare_log_level_arg,
        depth_anything_node
    ])