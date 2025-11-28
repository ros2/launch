# Copyright 2022 Open Source Robotics Foundation, Inc.
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

"""Tests for the PopEnvironment and PushEnvironment action classes."""

import os

from launch import LaunchContext
from launch.actions import PopEnvironment
from launch.actions import PushEnvironment

from temporary_environment import sandbox_environment_variables


@sandbox_environment_variables
def test_push_and_pop_environment_constructors():
    """Test the constructors for PopEnvironment and PushEnvironment classes."""
    PopEnvironment()
    PushEnvironment()


@sandbox_environment_variables
def test_push_and_pop_environment_execute():
    """Test the execute() of the PopEnvironment and PushEnvironment classes."""
    assert isinstance(os.environ, os._Environ)

    context = LaunchContext()

    # does not change empty state
    context.environment.clear()
    assert len(context.environment) == 0
    PushEnvironment().visit(context)
    PopEnvironment().visit(context)
    assert len(context.environment) == 0

    # does not change single env
    context.environment['foo'] = 'FOO'
    assert len(context.environment) == 1
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'FOO'
    PushEnvironment().visit(context)
    PopEnvironment().visit(context)
    assert len(context.environment) == 1
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'FOO'

    # does scope additions
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    PushEnvironment().visit(context)
    context.environment['foo'] = 'FOO'
    PopEnvironment().visit(context)
    assert len(context.environment) == 0

    # does scope modifications
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    context.environment['foo'] = 'FOO'
    PushEnvironment().visit(context)
    context.environment['foo'] = 'BAR'
    PopEnvironment().visit(context)
    assert len(context.environment) == 1
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'FOO'

    # does scope deletions
    context = LaunchContext()
    context.environment.clear()
    assert len(context.environment) == 0
    context.environment['foo'] = 'FOO'
    PushEnvironment().visit(context)
    del context.environment['foo']
    PopEnvironment().visit(context)
    assert len(context.environment) == 1
    assert 'foo' in context.environment
    assert context.environment['foo'] == 'FOO'

    # Pushing and popping the environment should not change the type of os.environ
    assert isinstance(os.environ, os._Environ)
