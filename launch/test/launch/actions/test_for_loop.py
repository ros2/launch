# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Tests for the ForLoop action."""

from typing import Callable
from typing import List
from typing import Optional

from launch import Action
from launch import LaunchContext
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import ForLoop
from launch.substitutions import LaunchConfiguration


def for_i(
    returned_entities: List[LaunchDescriptionEntity],
    i_collector: Optional[List[int]] = None,
) -> Callable[[int], Optional[List[LaunchDescriptionEntity]]]:
    def f(i: int):
        if i_collector is not None:
            i_collector.append(i)
        return returned_entities
    return f


def test_for_loop_constructors():
    """Test the constructors for the ForLoop class."""
    ForLoop('2', function=for_i([]))
    ForLoop(LaunchConfiguration('num'), function=for_i([]))


def test_for_loop_execute():
    """Test the execute() of the ForLoop class."""
    context = LaunchContext()
    i_values = []

    result = ForLoop('0', function=for_i([], i_values)).visit(context)
    assert len(result) == 0
    assert i_values == []
    i_values.clear()

    result = ForLoop('0', function=for_i(None, i_values)).visit(context)
    assert len(result) == 0
    assert i_values == []
    i_values.clear()

    result = ForLoop('2', function=for_i([], i_values)).visit(context)
    assert len(result) == 0
    assert i_values == [0, 1]
    i_values.clear()

    result = ForLoop('0', function=for_i([Action()], i_values)).visit(context)
    assert len(result) == 0
    assert i_values == []
    i_values.clear()

    result = ForLoop('2', function=for_i([Action()], i_values)).visit(context)
    assert len(result) == 2
    assert isinstance(result[0], Action)
    assert isinstance(result[1], Action)
    assert i_values == [0, 1]
    i_values.clear()

    # Use launch arg, first with default value then non-default value
    DeclareLaunchArgument('num', default_value='4').visit(context)
    result = ForLoop(
        LaunchConfiguration('num'),
        function=for_i([Action()], i_values),
    ).visit(context)
    assert len(result) == 4
    assert i_values == [0, 1, 2, 3]
    i_values.clear()
    context.launch_configurations['num'] = '5'
    result = ForLoop(
        LaunchConfiguration('num'),
        function=for_i([Action()], i_values),
    ).visit(context)
    assert len(result) == 5
    assert i_values == [0, 1, 2, 3, 4]
    i_values.clear()
    context.launch_configurations.clear()
