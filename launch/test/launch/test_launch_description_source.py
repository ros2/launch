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

"""Tests for the LaunchDescriptionSource class."""

import os

from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution

import pytest


def test_launch_description_source_constructors():
    """Test the constructors for LaunchDescriptionSource class."""
    LaunchDescriptionSource()
    LaunchDescriptionSource(LaunchDescription())
    LaunchDescriptionSource(LaunchDescription(), 'location')
    LaunchDescriptionSource(LaunchDescription(), 'location', 'method description')


def test_launch_description_source_methods():
    """Test the methods of the LaunchDescriptionSource class."""
    class MockLaunchContext:

        def perform_substitution(self, substitution):
            return substitution.perform(None)

    lds = LaunchDescriptionSource()
    with pytest.raises(RuntimeError):
        lds.get_launch_description(MockLaunchContext())

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld)
    assert lds.get_launch_description(MockLaunchContext()) == ld

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld, 'location')
    assert lds.get_launch_description(MockLaunchContext()) == ld
    assert lds.location == 'location'

    ld = LaunchDescription()
    lds = LaunchDescriptionSource(ld, 'location', 'method description')
    assert lds.get_launch_description(MockLaunchContext()) == ld
    assert lds.location == 'location'
    assert lds.method == 'method description'


def test_launch_description_source_location_expands_substitutions():
    """Test that location expands substitutions before get_launch_description()."""
    # A location made of substitutions that do not need a context is
    # returned expanded, not as a description of the substitutions.
    lds = LaunchDescriptionSource(
        LaunchDescription(),
        PathJoinSubstitution(['launch', 'launch_file.py']),
    )
    assert lds.location == os.path.join('launch', 'launch_file.py')

    # Same for the PythonLaunchDescriptionSource-style plain path used in
    # the issue: PathJoinSubstitution over plain components.
    lds = LaunchDescriptionSource(
        LaunchDescription(),
        PathJoinSubstitution([[TextSubstitution(text='some')], 'file.launch.py']),
    )
    assert lds.location == os.path.join('some', 'file.launch.py')


def test_launch_description_source_location_unexpandable_substitutions():
    """Test that location describes substitutions it cannot expand without a context."""
    # LaunchConfiguration has no value in an empty context and no default,
    # so the location cannot be expanded yet; it is described instead of
    # raising or returning an object repr.
    lds = LaunchDescriptionSource(
        LaunchDescription(),
        LaunchConfiguration('unset_launch_configuration'),
    )
    assert lds.location == "LaunchConfig('unset_launch_configuration')"

    # A location that mixes expandable and unexpandable substitutions also
    # falls back to a description.
    lds = LaunchDescriptionSource(
        LaunchDescription(),
        ['prefix-', LaunchConfiguration('unset_launch_configuration')],
    )
    assert 'LaunchConfig' in lds.location
    assert 'object at' not in lds.location
