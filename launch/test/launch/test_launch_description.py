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

"""Tests for the LaunchDescription class."""

import collections

from launch import Action
from launch import LaunchDescription
from launch import LaunchDescriptionEntity


def test_launch_description_constructors():
    """Test the constructors for LaunchDescription class."""
    LaunchDescription()
    LaunchDescription(None)
    LaunchDescription([])
    ld = LaunchDescription([LaunchDescriptionEntity()])
    assert len(ld.entities) == 1


def test_launch_description_add_things():
    """Test adding things to the LaunchDescription class."""
    ld = LaunchDescription()
    assert len(ld.entities) == 0
    ld.add_entity(LaunchDescription())
    assert len(ld.entities) == 1
    ld.add_action(Action())
    assert len(ld.entities) == 2


def test_launch_description_visit():
    """Test visiting entities in the LaunchDescription class."""
    ld = LaunchDescription([LaunchDescriptionEntity()])
    ld.add_action(Action())

    class MockLaunchContext:
        ...

    result = ld.visit(MockLaunchContext())
    assert isinstance(result, collections.Iterable)
    for entity in result:
        assert isinstance(entity, LaunchDescriptionEntity)
