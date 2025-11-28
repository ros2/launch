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

"""Tests for the ReplaceEnvironmentVariables action classes."""

import os

from launch import LaunchContext
from launch.actions import PopEnvironment
from launch.actions import PushEnvironment
from launch.actions import ReplaceEnvironmentVariables

from temporary_environment import sandbox_environment_variables


@sandbox_environment_variables
def test_replace_environment_constructors():
    """Test the constructors for ReplaceEnvironmentVariables class."""
    ReplaceEnvironmentVariables({})
    ReplaceEnvironmentVariables({'foo': 'bar', 'spam': 'eggs'})


@sandbox_environment_variables
def test_replace_environment_execute():
    """Test the execute() of the ReplaceEnvironmentVariables class."""
    assert isinstance(os.environ, os._Environ)

    # replaces empty state
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    ReplaceEnvironmentVariables({'foo': 'bar', 'spam': 'eggs'}).visit(context)
    assert len(context.environment) == 2
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'bar'
    assert 'spam' in context.environment
    assert context.environment['spam'] == 'eggs'

    # replaces non empty state
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    context.environment['quux'] = 'quuux'
    assert len(context.environment) == 1
    assert 'quux' in context.environment
    assert context.environment['quux'] == 'quuux'
    ReplaceEnvironmentVariables({'foo': 'bar', 'spam': 'eggs'}).visit(context)
    assert len(context.environment) == 2
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'bar'
    assert 'spam' in context.environment
    assert context.environment['spam'] == 'eggs'

    # replaces existing environment variable
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    context.environment['quux'] = 'quuux'
    assert len(context.environment) == 1
    assert 'quux' in context.environment
    assert context.environment['quux'] == 'quuux'
    ReplaceEnvironmentVariables({'quux': 'eggs'}).visit(context)
    assert len(context.environment) == 1
    assert 'quux' in context.environment
    assert context.environment['quux'] == 'eggs'

    # Replacing the environment should not change the type of os.environ
    assert isinstance(os.environ, os._Environ)

    # does not interfere with PopEnvironment and PushEnvironment action classes
    context = LaunchContext()
    context.environment.clear()
    context.environment['quux'] = 'quuux'
    assert len(context.environment) == 1
    assert 'quux' in context.environment
    assert context.environment['quux'] == 'quuux'
    PushEnvironment().visit(context)
    ReplaceEnvironmentVariables({'foo': 'bar', 'spam': 'eggs'}).visit(context)
    PopEnvironment().visit(context)
    assert len(context.environment) == 1
    assert 'quux' in context.environment
    assert context.environment['quux'] == 'quuux'
