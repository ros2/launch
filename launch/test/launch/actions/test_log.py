# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests for the Log action classes."""

from launch import LaunchContext
from launch.actions import Log
from launch.actions import LogDebug
from launch.actions import LogError
from launch.actions import LogInfo
from launch.actions import LogWarning
from launch.utilities import perform_substitutions

import pytest


def test_log_constructors():
    """Test the constructors for Log classes."""
    Log(msg='', level='INFO')
    Log(msg='', level='DEBUG')
    Log(msg='foo', level='WARNING')
    Log(msg=['foo', 'bar', 'baz'], level='ERROR')

    LogDebug(msg='')
    LogDebug(msg='foo')
    LogDebug(msg=['foo', 'bar', 'baz'])

    LogError(msg='')
    LogError(msg='foo')
    LogError(msg=['foo', 'bar', 'baz'])

    LogInfo(msg='')
    LogInfo(msg='foo')
    LogInfo(msg=['foo', 'bar', 'baz'])

    LogWarning(msg='')
    LogWarning(msg='foo')
    LogWarning(msg=['foo', 'bar', 'baz'])


def test_log_methods():
    """Test the methods of the LogInfo class."""
    launch_context = LaunchContext()

    log = Log(msg='', level='INFO')
    assert perform_substitutions(launch_context, log.msg) == ''

    log = Log(msg='foo', level='INFO')
    assert perform_substitutions(launch_context, log.msg) == 'foo'

    log = Log(msg=['foo', 'bar', 'baz'], level='INFO')
    assert perform_substitutions(launch_context, log.msg) == 'foobarbaz'

    log = Log(msg=['foo', 'bar', 'baz'], level=['I', 'N', 'F', 'O'])
    assert perform_substitutions(launch_context, log.level) == 'INFO'

    log = LogDebug(msg='')
    assert perform_substitutions(launch_context, log.level) == 'DEBUG'

    log = LogError(msg='')
    assert perform_substitutions(launch_context, log.level) == 'ERROR'

    log = LogInfo(msg='')
    assert perform_substitutions(launch_context, log.level) == 'INFO'

    log = LogWarning(msg='')
    assert perform_substitutions(launch_context, log.level) == 'WARNING'


def test_log_execute():
    """Test the execute (or visit) of the LogInfo class."""
    log = Log(msg='foo', level='ERROR')
    launch_context = LaunchContext()
    assert log.visit(launch_context) is None


def test_log_level_error():
    """Checks for error message to be raised given invalid level."""
    launch_context = LaunchContext()
    with pytest.raises(KeyError, match=r'Invalid log level*'):
        Log(msg='foo', level='foo').execute(launch_context)
