# Copyright 2021 Open Source Robotics Foundation, Inc.
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

"""Tests for the AppendEnvironmentVariable action."""

import os

from launch import LaunchContext
from launch.actions import AppendEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import TextSubstitution

from temporary_environment import sandbox_environment_variables


@sandbox_environment_variables
def test_append_environment_variable_constructor():
    """Test the constructor for the AppendEnvironmentVariable class."""
    AppendEnvironmentVariable('name', 'value')
    AppendEnvironmentVariable('name', 'value', separator='|')
    AppendEnvironmentVariable('name', 'value', prepend=False)
    AppendEnvironmentVariable('name', 'value', prepend=True)
    AppendEnvironmentVariable('name', 'value', prepend=True, separator='|')


@sandbox_environment_variables
def test_append_environment_variable_execute():
    """Test the execute() of the AppendEnvironmentVariable class."""
    context = LaunchContext()

    # Sets environment variable if it does not exist
    assert context.environment.get('NONEXISTENT_KEY') is None
    AppendEnvironmentVariable('NONEXISTENT_KEY', 'value').visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == 'value'
    # Same result if prepending is enabled
    del context.environment['NONEXISTENT_KEY']
    AppendEnvironmentVariable('NONEXISTENT_KEY', 'value', prepend=True).visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == 'value'

    # Appends to environment variable if it does exist
    AppendEnvironmentVariable('NONEXISTENT_KEY', 'another value').visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == 'value' + os.pathsep + 'another value'

    # Prepends to environment variable if it does exist and option is enabled
    AppendEnvironmentVariable('NONEXISTENT_KEY', 'some value', prepend=True).visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == \
        'some value' + os.pathsep + 'value' + os.pathsep + 'another value'

    # Can use an optional separator
    AppendEnvironmentVariable('NONEXISTENT_KEY', 'other value', separator='|').visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == \
        'some value' + os.pathsep + 'value' + os.pathsep + 'another value' + '|' + 'other value'

    # Appends/prepends with substitutions
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None
    AppendEnvironmentVariable(
        'ANOTHER_NONEXISTENT_KEY',
        EnvironmentVariable('NONEXISTENT_KEY'),
        prepend=TextSubstitution(text='false')).visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') == \
        'some value' + os.pathsep + 'value' + os.pathsep + 'another value' + '|' + 'other value'

    context.environment['ANOTHER_NONEXISTENT_KEY'] = 'abc'
    context.environment['SOME_SEPARATOR'] = '//'
    AppendEnvironmentVariable(
        'ANOTHER_NONEXISTENT_KEY',
        TextSubstitution(text='def'),
        separator=EnvironmentVariable('SOME_SEPARATOR'),
        prepend=TextSubstitution(text='yes')).visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') == 'def' + '//' + 'abc'
