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

"""Tests for the IncludeLaunchDescription action class."""

import os

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def test_include_launch_description_constructors():
    """Test the constructors for IncludeLaunchDescription class."""
    IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription()))


def test_include_launch_description_methods():
    """Test the methods of the IncludeLaunchDescription class."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    assert action.visit(LaunchContext()) == [ld]
    assert action.get_asyncio_future() is None

    ld2 = LaunchDescription([action])
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    assert action2.visit(LaunchContext()) == [ld2]
    assert action2.get_asyncio_future() is None


def test_include_launch_description_launch_file_location():
    """Test the ability of the IncludeLaunchDescription class to set the launch file location."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld, '<script>'))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    lc1 = LaunchContext()
    assert action.visit(lc1) == [ld]
    assert lc1.locals.current_launch_file_directory == '<script>'
    assert action.get_asyncio_future() is None

    this_file = os.path.abspath(__file__)
    ld2 = LaunchDescription()
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2, this_file))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    lc2 = LaunchContext()
    assert action2.visit(lc2) == [ld2]
    assert lc2.locals.current_launch_file_directory == os.path.dirname(this_file)
    assert action2.get_asyncio_future() is None
