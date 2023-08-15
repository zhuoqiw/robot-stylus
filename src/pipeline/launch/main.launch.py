"""
Generate a launch description.

This launch configuration gathers a bunch of node into a pipeline:
It parses the underlay config: .params.yaml and overlay config: params.yaml.
The overlay is updated to the underlay.
Several computation intensive nodes employee more workers.
Image related topics are passed around via intra-process communication.
Only pointers to image are copied to minimize CPU consumption.
"""

# Copyright 2019 InSoul Tech, Inc.
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
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():
    camera_pylon_node_l = ComposableNode(
        package='camera_pylon',
        plugin='camera_pylon::CameraPylon',
        name='camera_pylon_node_l',
        parameters=[{'workers': 1, 'serial': '22404673'}],
        remappings=[('~/grab', '/grab')],
        extra_arguments=[{'use_intra_process_comms': True}])

    camera_pylon_node_r = ComposableNode(
        package='camera_pylon',
        plugin='camera_pylon::CameraPylon',
        name='camera_pylon_node_r',
        parameters=[{'workers': 1, 'serial': '22935134'}],
        remappings=[('~/grab', '/grab')],
        extra_arguments=[{'use_intra_process_comms': True}])

    locate_stylus_node_l = ComposableNode(
        package='locate_stylus',
        plugin='locate_stylus::LocateStylus',
        name='locate_stylus_node_l',
        parameters=[{'workers': 1}],
        remappings=[('~/image', '/camera_pylon_node_l/image')],
        extra_arguments=[{'use_intra_process_comms': True}])

    locate_stylus_node_r = ComposableNode(
        package='locate_stylus',
        plugin='locate_stylus::LocateStylus',
        name='locate_stylus_node_r',
        parameters=[{'workers': 1}],
        remappings=[('~/image', '/camera_pylon_node_r/image')],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile = os.path.join(
        get_package_share_directory('reconstruct_pose'),
        'config',
        'params.yaml'
    )

    with open(configFile, 'r') as file:
        params = yaml.safe_load(file)['reconstruct_pose_node']['ros__parameters']

    reconstruct_pose_node = ComposableNode(
        package='reconstruct_pose',
        plugin='reconstruct_pose::ReconstructPose',
        parameters=[params],
        remappings=[('~/points_l', '/locate_stylus_node_l/points'),
                    ('~/points_r', '/locate_stylus_node_r/points')],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_pylon_node_l,
            camera_pylon_node_r,
            locate_stylus_node_l,
            locate_stylus_node_r,
            reconstruct_pose_node],
        output='screen')

    stylus = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '1',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'world',
            '--child-frame-id', 'stylus']
    )

    return launch.LaunchDescription([container, stylus])
