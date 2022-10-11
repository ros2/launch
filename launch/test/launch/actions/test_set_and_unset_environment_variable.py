# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetEnvironmentVariable and UnsetEnvironmentVariable action classes."""

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

from temporary_environment import sandbox_environment_variables


@sandbox_environment_variables
def test_set_and_unset_environment_variable_constructors():
    """Test the constructor for SetEnvironmentVariable and UnsetEnvironmentVariable classes."""
    SetEnvironmentVariable('name', 'value')
    UnsetEnvironmentVariable('name')


@sandbox_environment_variables
def test_set_and_unset_environment_variable_execute():
    """Test the execute() of the SetEnvironmentVariable and UnsetEnvironmentVariable classes."""
    context = LaunchContext()

    # can set and overwrite environment variables
    assert context.environment.get('NONEXISTENT_KEY') is None
    SetEnvironmentVariable('NONEXISTENT_KEY', 'value').visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == 'value'
    SetEnvironmentVariable('NONEXISTENT_KEY', 'ANOTHER_NONEXISTENT_KEY').visit(context)
    assert context.environment.get('NONEXISTENT_KEY') == 'ANOTHER_NONEXISTENT_KEY'

    # can unset environment variables
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None
    SetEnvironmentVariable('ANOTHER_NONEXISTENT_KEY', 'some value').visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') == 'some value'
    UnsetEnvironmentVariable('ANOTHER_NONEXISTENT_KEY').visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None

    # set and unset with substitutions
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None
    SetEnvironmentVariable(
        'ANOTHER_NONEXISTENT_KEY',
        EnvironmentVariable('NONEXISTENT_KEY')).visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') == 'ANOTHER_NONEXISTENT_KEY'
    UnsetEnvironmentVariable(EnvironmentVariable('NONEXISTENT_KEY')).visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None


def test_unset_nonexistent_key():
    """Test that the UnsetEnvironmentVariable class doesn't raise an exception."""
    context = LaunchContext()

    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None
    UnsetEnvironmentVariable('ANOTHER_NONEXISTENT_KEY').visit(context)
    assert context.environment.get('ANOTHER_NONEXISTENT_KEY') is None
