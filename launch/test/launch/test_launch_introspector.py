# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Tests for LaunchIntrospector formatting used by --print-description."""

from launch import LaunchDescription
from launch import LaunchIntrospector
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def test_introspector_shows_declare_launch_argument_details():
    """Declared arguments should show name and default, not a default object repr."""
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
    ])
    text = LaunchIntrospector().format_launch_description(ld)
    assert 'use_sim_time' in text
    assert 'false' in text
    assert 'Use simulation clock' in text
    assert 'DeclareLaunchArgument' in text
    assert 'object at 0x' not in text


def test_introspector_shows_opaque_function_name():
    """OpaqueFunction should identify the wrapped callable instead of an object id."""
    def generate_nodes(context):
        return []

    ld = LaunchDescription([
        OpaqueFunction(function=generate_nodes),
    ])
    text = LaunchIntrospector().format_launch_description(ld)
    assert 'generate_nodes' in text
    assert 'OpaqueFunction' in text
    assert 'LaunchDescription(1 entity)' in text
    assert 'object at 0x' not in text
