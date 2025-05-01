# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Tests for the ScopedIncludeLaunchDescription action class."""

import os

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionSource
# from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import PopEnvironment, PopLaunchConfigurations
from launch.actions import PushEnvironment, PushLaunchConfigurations
from launch.actions import ResetEnvironment, ResetLaunchConfigurations
from launch.actions import ScopedIncludeLaunchDescription

import pytest


def test_include_launch_description_constructors():
    """Test the constructors for ScopedIncludeLaunchDescription class."""
    ScopedIncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription()))
    ScopedIncludeLaunchDescription(
        LaunchDescriptionSource(LaunchDescription()),
        launch_arguments={'foo': 'FOO'}.items())


# TODO(SuperJappie08) Is this test necessary, my idea was check if filelocation does not leak
@pytest.mark.skip(reason='Not finished')
def test_scoped_include_launch_description_launch_file_location():
    """Test the ability of ScopedIncludeLaunchDescription to set the launch file location."""
    ld = LaunchDescription()
    action = ScopedIncludeLaunchDescription(LaunchDescriptionSource(ld, '<script>'))
    assert 'ScopedIncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    lc1 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action.visit(lc1) == [ld]
    assert lc1.locals.current_launch_file_directory == '<script>'
    assert action.get_asyncio_future() is None

    this_file = os.path.abspath(__file__)
    ld2 = LaunchDescription()
    action2 = ScopedIncludeLaunchDescription(LaunchDescriptionSource(ld2, this_file))
    assert 'ScopedIncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    lc2 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action2.visit(lc2) == [ld2]
    assert lc2.locals.current_launch_file_directory == os.path.dirname(this_file)
    assert action2.get_asyncio_future() is None


def test_scoped_include_launch_description_scoping():
    """Test for verifying scoping behavior of ScopedIncludeLaunchDescription."""
    ld_empty = LaunchDescription()
    action = ScopedIncludeLaunchDescription(LaunchDescriptionSource(ld_empty, '<script>'))
    assert 'ScopedIncludeLaunchDescription' in action.describe()

    # Verify the sub entity list
    sub_entities = action.describe_sub_entities()
    assert isinstance(sub_entities, list)
    assert len(sub_entities) == 7
    assert isinstance(sub_entities[0], PushLaunchConfigurations)
    assert isinstance(sub_entities[1], PushEnvironment)
    assert isinstance(sub_entities[2], ResetEnvironment)
    assert isinstance(sub_entities[3], ResetLaunchConfigurations)
    assert sub_entities[3]._ResetLaunchConfigurations__launch_configurations == {}
    assert isinstance(sub_entities[4], LaunchDescription)
    assert sub_entities[4] == ld_empty
    assert isinstance(sub_entities[5], PopEnvironment)
    assert isinstance(sub_entities[6], PopLaunchConfigurations)

    assert isinstance(action.describe_conditional_sub_entities(), list)
    lc1 = LaunchContext()

    # Empty inner into Empty outer
    # TODO(SuperJappie08): Update Comment
    # Result should only contain the launch description as there are no launch arguments.

    # lc1_pre_env = list(lc1.environment.items())
    # assert list(lc1.environment.items()) == lc1_pre_env
    res1 = action.visit(lc1)
    assert len(res1) == 3

    assert isinstance(res1[0], LaunchDescription)
    assert res1[0] == ld_empty
    assert res1[0].get_launch_arguments_with_include_launch_description_actions() == []

    assert isinstance(res1[1], PopEnvironment)
    assert isinstance(res1[2], PopLaunchConfigurations)

    # Non Empty Inner into empty outer
    inner_declare_argument = DeclareLaunchArgument('some_name', default_value='some_value')
    ld_non_empty = LaunchDescription([inner_declare_argument])
    action = ScopedIncludeLaunchDescription(LaunchDescriptionSource(ld_non_empty, '<script>'))
    assert 'ScopedIncludeLaunchDescription' in action.describe()

    # Verify the sub entity list
    sub_entities = action.describe_sub_entities()
    assert isinstance(sub_entities, list)
    assert len(sub_entities) == 7
    assert isinstance(sub_entities[0], PushLaunchConfigurations)
    assert isinstance(sub_entities[1], PushEnvironment)
    assert isinstance(sub_entities[2], ResetEnvironment)
    assert isinstance(sub_entities[3], ResetLaunchConfigurations)
    assert sub_entities[3]._ResetLaunchConfigurations__launch_configurations == {}
    assert isinstance(sub_entities[4], LaunchDescription)
    assert sub_entities[4] == ld_non_empty
    assert isinstance(sub_entities[5], PopEnvironment)
    assert isinstance(sub_entities[6], PopLaunchConfigurations)

    assert isinstance(action.describe_conditional_sub_entities(), list)

    lc2 = LaunchContext()
    # TODO: Finish NotImplemented()

# TODO(SuperJappie08) Add tests to verify behavior
