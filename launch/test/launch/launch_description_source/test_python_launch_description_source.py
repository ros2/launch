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

"""Tests for the PythonLaunchDescriptionSource class."""

import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import InvalidPythonLaunchFileError
from launch.launch_description_sources import PythonLaunchDescriptionSource

import pytest


def test_python_launch_description_source():
    """Test the PythonLaunchDescriptionSource class."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    simple_launch_file_path = os.path.join(this_dir, 'simple.launch.py')
    plds = PythonLaunchDescriptionSource(simple_launch_file_path)
    assert 'python launch file' in plds.method
    assert 'launch.substitutions.text_substitution.TextSubstitution' in plds.location
    ld = plds.get_launch_description(LaunchContext())
    assert plds.location == simple_launch_file_path
    assert 0 == len(ld.entities)

    with pytest.raises(InvalidPythonLaunchFileError):
        plds = PythonLaunchDescriptionSource(os.path.join(this_dir, 'loadable_python_module.py'))
        ld = plds.get_launch_description(LaunchContext())

    with pytest.raises(FileNotFoundError):
        plds = PythonLaunchDescriptionSource('does_not_exist')
        ld = plds.get_launch_description(LaunchContext())


def test_include_python():
    """Test PythonLaunchDescriptionSource with IncludeLaunchDescription action."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    simple_launch_file_path = os.path.join(this_dir, 'simple.launch.py')
    plds = PythonLaunchDescriptionSource(simple_launch_file_path)

    action = IncludeLaunchDescription(plds)
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    # Result should only contain a single launch description as there are no launch arguments.
    assert len(action.visit(LaunchContext())) == 1
    assert action.get_asyncio_future() is None
    assert len(action.launch_arguments) == 0

    assert action.launch_description_source.location == simple_launch_file_path


def test_include_python_implicit():
    """Test IncludeLaunchDescription with implicit inclusion of PythonLaunchDescriptionSource."""
    this_dir = os.path.dirname(os.path.abspath(__file__))
    simple_launch_file_path = os.path.join(this_dir, 'simple.launch.py')

    # pass in string instead of PythonLaunchDescriptionSource
    action = IncludeLaunchDescription(simple_launch_file_path)
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    # Result should only contain a single launch description as there are no launch arguments.
    assert len(action.visit(LaunchContext())) == 1
    assert action.get_asyncio_future() is None
    assert len(action.launch_arguments) == 0

    assert action.launch_description_source.location == simple_launch_file_path
