# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the Node Action."""

import os
import pathlib

from launch import LaunchDescription
from launch import LaunchService
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node


def test_launch_invalid_node():
    """Test launching an invalid node."""
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='nonexistent_package', node_executable='node', output='screen'),
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 != ls.run()


def test_launch_node():
    """Test launching a node."""
    ld = LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_py', node_executable='talker_qos', output='screen',
            arguments=['--number_of_cycles', '5'],
        ),
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()


def test_launch_node_with_parameters():
    """Test launching a node with parameters."""
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'example_parameters.yaml'
    # Pass parameter files to node in a variety of forms.
    # It is the same file because the objective is to test the different parameter types.
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    node_action = launch_ros.actions.Node(
        package='demo_nodes_py', node_executable='talker_qos', output='screen',
        arguments=['--number_of_cycles', '5'],
        parameters=[
            parameters_file_path,
            str(parameters_file_path),
            [EnvironmentVariable(name='FILE_PATH'), os.sep, 'example_parameters.yaml'],
        ],
    )
    ld = LaunchDescription([node_action])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()

    # Check the expanded parameters.
    expanded_parameters = node_action._Node__expanded_parameters
    assert len(expanded_parameters) == 3
    for i in range(3):
        assert expanded_parameters[i] == str(parameters_file_path)
