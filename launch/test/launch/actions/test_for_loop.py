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

"""Tests for the ForEach and ForLoop actions."""

from typing import Any
from typing import Callable
from typing import List
from typing import Mapping
from typing import Optional

from launch import Action
from launch import LaunchContext
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import ForEach
from launch.actions import ForLoop
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
import pytest


def for_each(
    returned_entities: List[LaunchDescriptionEntity],
    args_collector: Optional[List[Mapping[str, Any]]] = None,
) -> Callable[..., Optional[List[LaunchDescriptionEntity]]]:
    def f(**kwargs) -> List[LaunchDescriptionEntity]:
        if args_collector is not None:
            args_collector.append(kwargs)
        return returned_entities
    return f


def test_for_each_constructors():
    """Test the constructors for the ForEach class."""
    f = for_each([])
    action = ForEach('{};{}', function=f)
    assert len(action.input_values) == 1
    assert isinstance(action.input_values[0], TextSubstitution)
    assert action.input_values[0].text == '{};{}'
    assert action.function == f
    assert action.describe().startswith('ForEach')

    action = ForEach(TextSubstitution(text='{}'), function=f)
    assert len(action.input_values) == 1
    assert isinstance(action.input_values[0], TextSubstitution)
    assert action.input_values[0].text == '{}'
    assert action.function == f

    ForEach(LaunchConfiguration('config'), function=f)
    assert action.function == f


def test_for_each_execute():
    """Test the execute() of the ForEach class."""
    context = LaunchContext()
    iter_values = []

    # Should not iterate when input is empty
    result = ForEach('', function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == []
    iter_values.clear()

    result = ForEach('', function=for_each(None, iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == []
    iter_values.clear()

    result = ForEach('', function=for_each([Action()], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == []
    iter_values.clear()

    result = ForEach(';', function=for_each([Action()], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == []
    iter_values.clear()

    # Should still iterate with no args
    result = ForEach('{}', function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{}]
    iter_values.clear()

    result = ForEach('{}', function=for_each(None, iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{}]
    iter_values.clear()

    result = ForEach('{};', function=for_each(None, iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{}]
    iter_values.clear()

    result = ForEach('{};{}', function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{}, {}]
    iter_values.clear()

    result = ForEach(' {} ;  {}    ', function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{}, {}]
    iter_values.clear()

    result = ForEach('{};{};{}', function=for_each([Action()], iter_values)).visit(context)
    assert len(result) == 3
    assert isinstance(result[0], Action)
    assert isinstance(result[1], Action)
    assert isinstance(result[2], Action)
    assert iter_values == [{}, {}, {}]
    iter_values.clear()

    # Normal case
    result = ForEach("{name: 'a'};{name: 'b'}", function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == [{'name': 'a'}, {'name': 'b'}]
    iter_values.clear()

    result = ForEach(
        "{name: 'a'};{name: 'b'}",
        function=for_each(None, iter_values),
    ).visit(context)
    assert len(result) == 0
    assert iter_values == [{'name': 'a'}, {'name': 'b'}]
    iter_values.clear()

    result = ForEach(
        "{name: 'a'};{name: 'b'}",
        function=for_each([Action()], iter_values),
    ).visit(context)
    assert len(result) == 2
    assert isinstance(result[0], Action)
    assert isinstance(result[1], Action)
    assert iter_values == [{'name': 'a'}, {'name': 'b'}]
    iter_values.clear()


def test_for_each_execute_substitution():
    context = LaunchContext()
    iter_values = []

    # Text
    result = ForEach(TextSubstitution(text=''), function=for_each([], iter_values)).visit(context)
    assert len(result) == 0
    assert iter_values == []
    iter_values.clear()

    # Launch arg, first with default value then non-default value
    DeclareLaunchArgument('config', default_value='{id: 42};{id: 27}').visit(context)
    result = ForEach(
        LaunchConfiguration('config'),
        function=for_each([Action()], iter_values),
    ).visit(context)
    assert len(result) == 2
    assert iter_values == [{'id': 42}, {'id': 27}]
    iter_values.clear()
    context.launch_configurations['config'] = '{id: 6};{id: 9};{id: 420}'
    result = ForEach(
        LaunchConfiguration('config'),
        function=for_each([Action()], iter_values),
    ).visit(context)
    assert len(result) == 3
    assert iter_values == [{'id': 6}, {'id': 9}, {'id': 420}]
    iter_values.clear()
    context.launch_configurations.clear()

    # Python expression
    DeclareLaunchArgument('num', default_value='3').visit(context)
    result = ForEach(
        PythonExpression(
            ["';'.join([str({'num': i * 2}) for i in range(", LaunchConfiguration('num'), ')])'],
        ),
        function=for_each([Action(), Action()], iter_values),
    ).visit(context)
    assert len(result) == 6
    assert isinstance(result[0], Action)
    assert isinstance(result[1], Action)
    assert isinstance(result[2], Action)
    assert isinstance(result[3], Action)
    assert isinstance(result[4], Action)
    assert isinstance(result[5], Action)
    assert iter_values == [{'num': 0}, {'num': 2}, {'num': 4}]
    iter_values.clear()
    context.launch_configurations.clear()


def for_each_args(
    returned_entities: List[LaunchDescriptionEntity],
    args_collector: Optional[List[Mapping[str, Any]]] = None,
) -> Callable[..., Optional[List[LaunchDescriptionEntity]]]:
    def f(a: str, b: int, c: List[int]) -> List[LaunchDescriptionEntity]:
        if args_collector is not None:
            args_collector.append({'a': a, 'b': b, 'c': c})
        return returned_entities
    return f


def for_each_args_kwargs(
    returned_entities: List[LaunchDescriptionEntity],
    args_collector: Optional[List[Mapping[str, Any]]] = None,
) -> Callable[..., Optional[List[LaunchDescriptionEntity]]]:
    def f(a: str, *, b: int, c: List[int]) -> List[LaunchDescriptionEntity]:
        if args_collector is not None:
            args_collector.append({'a': a, 'b': b, 'c': c})
        return returned_entities
    return f


def for_each_args_kwargs_default(
    returned_entities: List[LaunchDescriptionEntity],
    args_collector: Optional[List[Mapping[str, Any]]] = None,
) -> Callable[..., Optional[List[LaunchDescriptionEntity]]]:
    def f(a: str, *, b: int, c: List[int] = [4, 2, 0]) -> List[LaunchDescriptionEntity]:
        if args_collector is not None:
            args_collector.append({'a': a, 'b': b, 'c': c})
        return returned_entities
    return f


def test_for_each_execute_args():
    context = LaunchContext()
    iter_values = []

    # Order of items in YAML dicts does not matter
    result = ForEach(
        '{a: 1, b: 2, c: 3};{c: 1, a: 2, b: 3}',
        function=for_each([], iter_values),
    ).visit(context)
    assert len(result) == 0
    assert iter_values == [{'a': 1, 'b': 2, 'c': 3}, {'a': 2, 'b': 3, 'c': 1}]
    iter_values.clear()

    # Using callback with args
    result = ForEach(
        "{a: 'tw', b: 1, c: []};{c: [4], a: 'll', b: 2}",
        function=for_each_args([], iter_values),
    ).visit(context)
    assert len(result) == 0
    assert iter_values == [{'a': 'tw', 'b': 1, 'c': []}, {'a': 'll', 'b': 2, 'c': [4]}]
    iter_values.clear()

    # Using callback with args and kwargs
    result = ForEach(
        "{a: 'tw', b: 1, c: []};{c: [4], a: 'll', b: 2}",
        function=for_each_args_kwargs([], iter_values),
    ).visit(context)
    assert len(result) == 0
    assert iter_values == [{'a': 'tw', 'b': 1, 'c': []}, {'a': 'll', 'b': 2, 'c': [4]}]
    iter_values.clear()

    # Using callback with args, kwargs, and default values
    result = ForEach(
        "{a: 'tw', b: 1, c: []};{a: 'll', b: 2}",
        function=for_each_args_kwargs_default([], iter_values),
    ).visit(context)
    assert len(result) == 0
    assert iter_values == [{'a': 'tw', 'b': 1, 'c': []}, {'a': 'll', 'b': 2, 'c': [4, 2, 0]}]
    iter_values.clear()


def for_i(
    returned_entities: List[LaunchDescriptionEntity],
    i_collector: Optional[List[int]] = None,
) -> Callable[[int], Optional[List[LaunchDescriptionEntity]]]:
    def f(i: int) -> List[LaunchDescriptionEntity]:
        if i_collector is not None:
            i_collector.append(i)
        return returned_entities
    return f


def test_for_loop_constructors():
    """Test the constructors for the ForLoop class."""
    f = for_i([])
    action = ForLoop('2', function=f, name='my-for-loop')
    assert len(action.length) == 1
    assert isinstance(action.length[0], TextSubstitution)
    assert action.length[0].text == '2'
    assert action.function == f
    assert action.name == 'my-for-loop'
    assert action.describe().startswith('ForLoop')

    action = ForLoop(LaunchConfiguration('num'), function=f)
    assert action.function == f


def test_for_loop_execute():
    """Test the execute() of the ForLoop class."""
    context = LaunchContext()
    i_values = []

    # Empty input
    with pytest.raises(ValueError):
        ForLoop('', function=for_i([], i_values)).visit(context)

    # No iterations
    result = ForLoop('0', function=for_i([], i_values)).visit(context)
    assert len(result) == 1
    assert isinstance(result[0], ForEach)
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 0
    assert i_values == []
    i_values.clear()

    result = ForLoop('0', function=for_i(None, i_values)).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 0
    assert i_values == []
    i_values.clear()

    # Normal case
    result = ForLoop('2', function=for_i([], i_values)).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 0
    assert i_values == [0, 1]
    i_values.clear()

    result = ForLoop('0', function=for_i([Action()], i_values)).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 0
    assert i_values == []
    i_values.clear()

    result = ForLoop('2', function=for_i([Action()], i_values)).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 2
    assert isinstance(result_for_each[0], Action)
    assert isinstance(result_for_each[1], Action)
    assert i_values == [0, 1]
    i_values.clear()


def test_for_loop_execute_substitutions():
    context = LaunchContext()
    i_values = []

    # Launch arg, first with default value then non-default value
    DeclareLaunchArgument('num', default_value='4').visit(context)
    result = ForLoop(
        LaunchConfiguration('num'),
        function=for_i([Action()], i_values),
    ).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 4
    assert i_values == [0, 1, 2, 3]
    i_values.clear()
    context.launch_configurations['num'] = '5'
    result = ForLoop(
        LaunchConfiguration('num'),
        function=for_i([Action()], i_values),
    ).visit(context)
    assert len(result) == 1
    result_for_each = result[0].visit(context)
    assert len(result_for_each) == 5
    assert i_values == [0, 1, 2, 3, 4]
    i_values.clear()
    context.launch_configurations.clear()
