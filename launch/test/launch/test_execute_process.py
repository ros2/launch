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

"""Tests for the ExecuteProcess Action."""

import os
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch.actions.opaque_function import OpaqueFunction

import pytest


@pytest.mark.parametrize('test_input,expected', [
    (None, [True, False]),
    ({'TEST_NEW_ENV': '2'}, [False, True])
])
def test_execute_process_with_env(test_input, expected):
    """Test launching a process with an environment variable."""
    os.environ['TEST_CHANGE_CURRENT_ENV'] = '1'
    additional_env = {'TEST_PROCESS_WITH_ENV': 'Hello World'}
    executable = ExecuteProcess(
        cmd=[sys.executable, 'TEST_PROCESS_WITH_ENV'],
        output='screen',
        env=test_input,
        additional_env=additional_env
    )
    ld = LaunchDescription([executable])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    env = executable.process_details['env']
    assert env['TEST_PROCESS_WITH_ENV'] == 'Hello World'
    assert ('TEST_CHANGE_CURRENT_ENV' in env) is expected[0]
    if expected[0]:
        assert env['TEST_CHANGE_CURRENT_ENV'] == '1'
    assert ('TEST_NEW_ENV' in env) is expected[1]
    if expected[1]:
        assert env['TEST_NEW_ENV'] == '2'


def test_execute_process_with_on_exit_behavior():
    """Test a process' on_exit callback and actions are processed."""
    def on_exit_callback(event, context):
        on_exit_callback.called = True
    on_exit_callback.called = False

    executable_with_on_exit_callback = ExecuteProcess(
        cmd=[sys.executable, '-c', "print('callback')"],
        output='screen', on_exit=on_exit_callback
    )
    assert len(executable_with_on_exit_callback.get_sub_entities()) == 0

    def on_exit_function(context):
        on_exit_function.called = True
    on_exit_function.called = False
    on_exit_action = OpaqueFunction(function=on_exit_function)
    executable_with_on_exit_action = ExecuteProcess(
        cmd=[sys.executable, '-c', "print('action')"],
        output='screen', on_exit=[on_exit_action]
    )
    assert executable_with_on_exit_action.get_sub_entities() == [on_exit_action]

    ld = LaunchDescription([
        executable_with_on_exit_callback,
        executable_with_on_exit_action
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert on_exit_callback.called
    assert on_exit_function.called
