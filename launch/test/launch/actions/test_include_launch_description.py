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
from pathlib import Path

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import PopEnvironment
from launch.actions import PopLaunchConfigurations
from launch.actions import PushEnvironment
from launch.actions import PushLaunchConfigurations
from launch.actions import ResetLaunchConfigurations
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFile
from launch.substitutions import ThisLaunchFileDir
from launch.utilities import perform_substitutions

import pytest

from temporary_environment import sandbox_environment_variables


def test_include_launch_description_constructors():
    """Test the constructors for IncludeLaunchDescription class."""
    IncludeLaunchDescription(LaunchDescriptionSource(LaunchDescription()))
    IncludeLaunchDescription(
        LaunchDescriptionSource(LaunchDescription()),
        launch_arguments={'foo': 'FOO'}.items())


def test_include_launch_description_methods():
    """Test the methods of the IncludeLaunchDescription class."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    # Result should only contain the launch description as there are no launch arguments.
    assert action.visit(LaunchContext())[0] == ld
    assert action.get_asyncio_future() is None
    assert len(action.launch_arguments) == 0

    ld2 = LaunchDescription([action])
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    # Result should only contain the launch description as there are no launch arguments.
    assert action2.visit(LaunchContext())[0] == ld2
    assert action2.get_asyncio_future() is None
    assert len(action2.launch_arguments) == 0


def test_include_launch_description_launch_file_location():
    """Test the ability of the IncludeLaunchDescription class to set the launch file location."""
    ld = LaunchDescription()
    action = IncludeLaunchDescription(LaunchDescriptionSource(ld, '<script>'))
    assert 'IncludeLaunchDescription' in action.describe()
    assert isinstance(action.describe_sub_entities(), list)
    assert isinstance(action.describe_conditional_sub_entities(), list)
    lc1 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action.visit(lc1)[0] == ld
    assert lc1.locals.current_launch_file_directory == '<script>'
    assert action.get_asyncio_future() is None

    this_file = Path(__file__).absolute()
    ld2 = LaunchDescription()
    action2 = IncludeLaunchDescription(LaunchDescriptionSource(ld2, this_file))
    assert 'IncludeLaunchDescription' in action2.describe()
    assert isinstance(action2.describe_sub_entities(), list)
    assert isinstance(action2.describe_conditional_sub_entities(), list)
    lc2 = LaunchContext()
    # Result should only contain the launch description as there are no launch arguments.
    assert action2.visit(lc2)[0] == ld2
    assert lc2.locals.current_launch_file_directory == str(this_file.parent)
    assert action2.get_asyncio_future() is None


def test_include_launch_description_launch_file_dir_location_scoped():
    """Test that the launch file name & dir locals are scoped to the included launch file."""
    # Rely on the test launch files to set environment variables with
    # ThisLaunchFile()/ThisLaunchFileDir() to make testing easier
    parent_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'launch')
    parent_launch_file = os.path.join(parent_dir, 'parent_launch_file_dir.launch.py')
    included_dir = os.path.join(parent_dir, 'included')
    included_launch_file = os.path.join(included_dir, 'launch_file_dir.launch.py')

    # The current launch file/dir context locals should be scoped to the included launch file
    ld = LaunchDescription([IncludeLaunchDescription(parent_launch_file)])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    lc = ls.context
    assert lc.environment.get('Before_ThisLaunchFile') == parent_launch_file
    assert lc.environment.get('Before_ThisLaunchFileDir') == parent_dir
    assert lc.environment.get('Included_ThisLaunchFile') == included_launch_file
    assert lc.environment.get('Included_ThisLaunchFileDir') == included_dir
    assert lc.environment.get('After_ThisLaunchFile') == parent_launch_file
    assert lc.environment.get('After_ThisLaunchFileDir') == parent_dir

    # The launch file/dir context locals should be completely removed after the first included
    # (parent) launch file, because at that point we're in a launch script and not a launch file,
    # and therefore these substitutions should raise an error
    ld2 = LaunchDescription([
        IncludeLaunchDescription(parent_launch_file),
        SetEnvironmentVariable('Outside_ThisLaunchFile', ThisLaunchFile()),
        SetEnvironmentVariable('Outside_ThisLaunchFileDir', ThisLaunchFileDir()),
    ])
    ls2 = LaunchService()
    ls2.include_launch_description(ld2)
    assert 1 == ls2.run()

    # The non-launch file/dir context locals should not be scoped to the included launch file
    def assert_unscoped_context_local(context: LaunchContext):
        assert context.locals.included_local == 'context_local_value'

    ld3 = LaunchDescription([
        IncludeLaunchDescription(parent_launch_file),
        OpaqueFunction(function=assert_unscoped_context_local),
    ])
    ls3 = LaunchService()
    ls3.include_launch_description(ld3)
    assert 0 == ls3.run()


def test_include_launch_description_launch_arguments():
    """Test the interactions between declared launch arguments and IncludeLaunchDescription."""
    # test that arguments are set when given, even if they are not declared
    ld1 = LaunchDescription([])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1),
        launch_arguments={'foo': 'FOO'}.items(),
    )
    assert len(action1.launch_arguments) == 1
    lc1 = LaunchContext()
    result1 = action1.visit(lc1)
    assert len(result1) == 3
    assert isinstance(result1[0], SetLaunchConfiguration)
    assert perform_substitutions(lc1, result1[0].name) == 'foo'
    assert perform_substitutions(lc1, result1[0].value) == 'FOO'
    assert result1[1] == ld1

    # test that a declared argument that is not provided raises an error
    ld2 = LaunchDescription([DeclareLaunchArgument('foo')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2)
    )
    lc2 = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo2:
        action2.visit(lc2)
    assert 'Included launch description missing required argument' in str(excinfo2.value)

    # test that a declared argument that is not provided raises an error, but with other args set
    ld2 = LaunchDescription([DeclareLaunchArgument('foo')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'not_foo': 'NOT_FOO'}.items(),
    )
    lc2 = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo2:
        action2.visit(lc2)
    assert 'Included launch description missing required argument' in str(excinfo2.value)
    assert 'not_foo' in str(excinfo2.value)

    # test that a declared argument with a default value that is not provided does not raise
    ld2 = LaunchDescription([DeclareLaunchArgument('foo', default_value='FOO')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2)
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that default arguments in nested IncludeLaunchDescription actions do not raise
    ld1 = LaunchDescription([DeclareLaunchArgument('foo', default_value='FOO')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1),
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that provided launch arguments of nested IncludeLaunchDescription actions do not raise
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1), launch_arguments={'foo': 'FOO'}.items(),
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that arguments can not be passed from the parent launch description
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo': 'FOO', 'foo2': 'FOO2'}.items(),
    )
    ld3 = LaunchDescription([action2])
    ls = LaunchService()
    ls.include_launch_description(ld3)
    assert 1 == ls.run()

    # Test that arguments can be redeclared in the parent launch description
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription([action1, DeclareLaunchArgument('foo'), DeclareLaunchArgument('foo2')])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo': 'FOO', 'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)

    # Test that arguments after a ResetLaunchConfigurations action are not checked
    ld1 = LaunchDescription([DeclareLaunchArgument('foo')])
    action1 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld1)
    )
    ld2 = LaunchDescription(
        [
            DeclareLaunchArgument('foo2'),
            ResetLaunchConfigurations(),
            SetLaunchConfiguration('foo', 'asd'),
            action1])
    action2 = IncludeLaunchDescription(
        LaunchDescriptionSource(ld2),
        launch_arguments={'foo2': 'FOO2'}.items(),
    )
    lc2 = LaunchContext()
    action2.visit(lc2)


@sandbox_environment_variables
def test_include_launch_description_scoped_execute():
    """Test scoped=True: Push/Pop wrapping, forwarding, and isolation of launch configurations."""
    ld_child = LaunchDescription([])
    action = IncludeLaunchDescription(
        LaunchDescriptionSource(ld_child),
        launch_arguments={'bar': 'BAR'}.items(),
        scoped=True,
    )

    lc = LaunchContext()
    lc.launch_configurations['foo'] = 'FOO'

    result = action.visit(lc)

    # Expected: Push, Push, SetLaunchConfig, LaunchDescription, OpaqueFunction, Pop, Pop
    assert len(result) == 7
    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], PushEnvironment)
    assert isinstance(result[2], SetLaunchConfiguration)
    assert result[3] == ld_child
    assert isinstance(result[4], OpaqueFunction)
    assert isinstance(result[5], PopEnvironment)
    assert isinstance(result[6], PopLaunchConfigurations)

    # Step through and verify intermediate state
    result[0].visit(lc)  # PushLaunchConfigurations
    assert lc.launch_configurations['foo'] == 'FOO'  # forwarded to child scope

    result[1].visit(lc)  # PushEnvironment

    result[2].visit(lc)  # SetLaunchConfiguration('bar', 'BAR')
    assert lc.launch_configurations['bar'] == 'BAR'
    assert lc.launch_configurations['foo'] == 'FOO'  # still visible

    # Simulate what the child launch description would do
    lc.launch_configurations['baz'] = 'BAZ'
    assert lc.launch_configurations['baz'] == 'BAZ'

    # result[3] (LaunchDescription) and result[4] (OpaqueFunction) skipped — they don't affect
    # launch_configurations directly in this test

    result[5].visit(lc)  # PopEnvironment
    result[6].visit(lc)  # PopLaunchConfigurations
    # After pop, child's configs are gone, parent's are restored
    assert lc.launch_configurations['foo'] == 'FOO'
    assert 'baz' not in lc.launch_configurations
    assert 'bar' not in lc.launch_configurations
    assert len(lc.launch_configurations) == 1


@sandbox_environment_variables
def test_include_launch_description_unscoped_execute():
    """Test scoped=False (default): no Push/Pop, configurations leak to parent."""
    ld_child = LaunchDescription([])
    action = IncludeLaunchDescription(
        LaunchDescriptionSource(ld_child),
        launch_arguments={'bar': 'BAR'}.items(),
    )

    lc = LaunchContext()
    lc.launch_configurations['foo'] = 'FOO'

    result = action.visit(lc)

    # Expected: SetLaunchConfig, LaunchDescription, OpaqueFunction (no Push/Pop)
    assert len(result) == 3
    assert isinstance(result[0], SetLaunchConfiguration)
    assert result[1] == ld_child
    assert isinstance(result[2], OpaqueFunction)
    assert not any(isinstance(r, PushLaunchConfigurations) for r in result)
    assert not any(isinstance(r, PopLaunchConfigurations) for r in result)

    # Step through
    result[0].visit(lc)  # SetLaunchConfiguration('bar', 'BAR')
    assert lc.launch_configurations['bar'] == 'BAR'
    assert lc.launch_configurations['foo'] == 'FOO'  # untouched

    # After all actions, bar persists — it leaked to the parent scope
    assert len(lc.launch_configurations) == 2
    assert lc.launch_configurations['bar'] == 'BAR'


@sandbox_environment_variables
def test_include_launch_description_scoped_isolates_environment():
    """Test scoped=True: environment variable changes do not leak to parent."""
    ld_child = LaunchDescription([])
    action = IncludeLaunchDescription(
        LaunchDescriptionSource(ld_child),
        scoped=True,
    )

    lc = LaunchContext()
    assert 'env_foo' not in lc.environment

    result = action.visit(lc)

    assert isinstance(result[0], PushLaunchConfigurations)
    assert isinstance(result[1], PushEnvironment)

    result[0].visit(lc)  # PushLaunchConfigurations
    result[1].visit(lc)  # PushEnvironment

    # Simulate child setting an environment variable
    lc.environment['env_foo'] = 'FOO'
    assert lc.environment['env_foo'] == 'FOO'

    assert isinstance(result[-2], PopEnvironment)
    assert isinstance(result[-1], PopLaunchConfigurations)

    result[-2].visit(lc)  # PopEnvironment
    assert 'env_foo' not in lc.environment  # rolled back

    result[-1].visit(lc)  # PopLaunchConfigurations


@sandbox_environment_variables
def test_include_launch_description_unscoped_leaks_environment():
    """Test scoped=False (default): environment variable changes leak to parent."""
    ld_child = LaunchDescription([])
    action = IncludeLaunchDescription(
        LaunchDescriptionSource(ld_child),
    )

    lc = LaunchContext()
    result = action.visit(lc)

    # No Push/Pop — environment mutations persist
    assert len(result) == 2  # LaunchDescription, OpaqueFunction (no launch_arguments)

    # Simulate child setting an environment variable
    lc.environment['env_foo'] = 'FOO'

    # After all actions, the env var persists — no Pop to roll it back
    assert lc.environment['env_foo'] == 'FOO'


@sandbox_environment_variables
def test_include_launch_description_scoped_with_overwrite():
    """Test scoped=True: child overwrites parent config, but parent value is restored after pop."""
    ld_child = LaunchDescription([])
    action = IncludeLaunchDescription(
        LaunchDescriptionSource(ld_child),
        launch_arguments={'foo': 'OOF'}.items(),
        scoped=True,
    )

    lc = LaunchContext()
    lc.launch_configurations['foo'] = 'FOO'
    lc.launch_configurations['bar'] = 'BAR'

    result = action.visit(lc)

    result[0].visit(lc)  # PushLaunchConfigurations
    assert lc.launch_configurations['foo'] == 'FOO'  # copied to new scope
    assert lc.launch_configurations['bar'] == 'BAR'  # forwarded

    result[1].visit(lc)  # PushEnvironment

    result[2].visit(lc)  # SetLaunchConfiguration('foo', 'OOF')
    assert lc.launch_configurations['foo'] == 'OOF'  # overwritten in child scope
    assert lc.launch_configurations['bar'] == 'BAR'  # untouched

    result[-2].visit(lc)  # PopEnvironment
    result[-1].visit(lc)  # PopLaunchConfigurations
    assert lc.launch_configurations['foo'] == 'FOO'  # restored
    assert lc.launch_configurations['bar'] == 'BAR'  # still there
    assert len(lc.launch_configurations) == 2


def test_include_python():
    """Test including Python, with and without explicit PythonLaunchDescriptionSource."""
    this_dir = Path(__file__).parent
    simple_launch_file_path = this_dir.parent / 'launch_description_source' / 'simple.launch.py'

    # Explicitly construct with PythonLaunchDescriptionSource
    plds = PythonLaunchDescriptionSource(simple_launch_file_path)
    action0 = IncludeLaunchDescription(plds)

    # Construct action with path instead of PythonLaunchDescriptionSource object
    action1 = IncludeLaunchDescription(simple_launch_file_path)

    # The two actions should be equivalent
    for action in [action0, action1]:
        assert 'IncludeLaunchDescription' in action.describe()
        assert isinstance(action.describe_sub_entities(), list)
        assert isinstance(action.describe_conditional_sub_entities(), list)
        # Result should only contain a single launch description (+ internal action) as there are
        # no launch arguments.
        assert len(action.visit(LaunchContext())) == 2
        assert action.get_asyncio_future() is None
        assert len(action.launch_arguments) == 0

        assert action.launch_description_source.location == str(simple_launch_file_path)
