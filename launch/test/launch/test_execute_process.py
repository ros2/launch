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
import platform
import signal
import sys

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import SetLaunchConfiguration
from launch.actions.emit_event import EmitEvent
from launch.actions.execute_process import ExecuteProcess
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.register_event_handler import RegisterEventHandler
from launch.actions.shutdown_action import Shutdown
from launch.actions.timer_action import TimerAction
from launch.event_handlers.on_process_start import OnProcessStart
from launch.events.shutdown import Shutdown as ShutdownEvent
from launch.substitutions.launch_configuration import LaunchConfiguration

import osrf_pycommon

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


def test_execute_process_shutdown():
    """Test shutting down a process in (non)interactive settings."""
    def on_exit(event, ctx):
        on_exit.returncode = event.returncode

    def generate_launch_description():
        process_action = ExecuteProcess(
            cmd=[sys.executable, '-S', '-c', 'input()'],
            sigterm_timeout='1',  # shorten timeouts
            on_exit=on_exit
        )
        # Launch process and emit shutdown event as if
        # launch had received a SIGINT
        return LaunchDescription([
            process_action,
            RegisterEventHandler(event_handler=OnProcessStart(
                target_action=process_action,
                on_start=[
                    EmitEvent(event=ShutdownEvent(
                        reason='none',
                        due_to_sigint=True
                    ))
                ]
            ))
        ])

    ls = LaunchService(noninteractive=True)
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    if platform.system() != 'Windows':
        assert on_exit.returncode == -signal.SIGINT  # Got SIGINT
    else:
        assert on_exit.returncode != 0  # Process terminated

    ls = LaunchService()  # interactive
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    if platform.system() != 'Windows':
        # Assume interactive Ctrl+C (i.e. SIGINT to process group)
        assert on_exit.returncode == -signal.SIGTERM  # Got SIGTERM
    else:
        assert on_exit.returncode != 0  # Process terminated


def test_execute_process_with_respawn():
    """Test launching a process with a respawn and respawn_delay attribute."""
    def on_exit_callback(event, context):
        on_exit_callback.called_count = on_exit_callback.called_count + 1
    on_exit_callback.called_count = 0

    respawn_delay = 2.0
    shutdown_time = 3.0   # to shutdown the launch service, so that the process only respawn once
    expected_called_count = 2   # normal exit and respawn exit

    def generate_launch_description():
        return LaunchDescription([

            ExecuteProcess(
                cmd=[sys.executable, '-c', "print('action')"],
                respawn=True, respawn_delay=respawn_delay, on_exit=on_exit_callback
            ),

            TimerAction(
                period=shutdown_time,
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
        ])

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    assert expected_called_count == on_exit_callback.called_count


def test_execute_process_with_respawn_substitution():
    """Test launching a process with a respawn substitution and respawn_delay attribute."""
    def on_exit_callback(event, context):
        on_exit_callback.called_count = on_exit_callback.called_count + 1

    respawn_delay = 2.0
    shutdown_time = 3.0  # to shutdown the launch service, so that the process only respawn once

    def generate_launch_description():

        test_process = ExecuteProcess(
            cmd=[sys.executable, '-c', "print('action')"],
            respawn=LaunchConfiguration('respawn'),
            respawn_delay=respawn_delay, on_exit=on_exit_callback,
        )

        ld = LaunchDescription([
            test_process,
            TimerAction(
                period=shutdown_time,
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
        ])

        return ld

    ls = LaunchService()
    ls.context.launch_configurations['respawn'] = 'False'
    ls.include_launch_description(generate_launch_description())
    on_exit_callback.called_count = 0
    expected_called_count = 1  # only normal exit
    assert 0 == ls.run()
    assert expected_called_count == on_exit_callback.called_count

    ls = LaunchService()
    ls.context.launch_configurations['respawn'] = 'True'
    on_exit_callback.called_count = 0
    expected_called_count = 2  # normal exit and respawn exit
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    assert expected_called_count == on_exit_callback.called_count


def test_execute_process_with_respawn_max_retries():
    """Test launching a process with respawn_max_retries attribute."""
    def on_exit_callback(event, context):
        on_exit_callback.called_count += 1
        if on_exit_callback.called_count == expected_called_count:
            timer = TimerAction(
                period=2.,   # wait to verify if the process continues to respawn itself
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
            timer.execute(context)
    on_exit_callback.called_count = 0

    respawn_max_retries = 2   # we want the process to respawn this amount of times
    expected_called_count = 3   # normal exit + respawn_max_retries exits
    shutdown_time = 10.0   # security timer to kill the process

    def generate_launch_description():
        return LaunchDescription([

            ExecuteProcess(
                cmd=[sys.executable, '-c', "print('action')"],
                respawn=True, respawn_max_retries=respawn_max_retries,
                on_exit=on_exit_callback
            ),

            TimerAction(
                period=shutdown_time,
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
        ])

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()
    assert expected_called_count == on_exit_callback.called_count


def test_execute_process_prefix_filter_match():
    lc = LaunchContext()
    lc._set_asyncio_loop(osrf_pycommon.process_utils.get_loop())

    SetLaunchConfiguration('launch-prefix', 'time').visit(lc)
    assert len(lc.launch_configurations) == 1
    SetLaunchConfiguration(
        'launch-prefix-filter',
        f'{os.path.basename(sys.executable)}').visit(lc)
    assert len(lc.launch_configurations) == 2

    test_process = ExecuteProcess(
        cmd=[sys.executable, '-c', "print('action')"],
        output='screen'
    )

    test_process.execute(lc)
    assert 'time' in test_process.process_details['cmd']


def test_execute_process_prefix_filter_no_match():
    lc = LaunchContext()
    lc._set_asyncio_loop(osrf_pycommon.process_utils.get_loop())

    SetLaunchConfiguration('launch-prefix', 'time').visit(lc)
    assert len(lc.launch_configurations) == 1
    SetLaunchConfiguration(
        'launch-prefix-filter', 'no-match').visit(lc)
    assert len(lc.launch_configurations) == 2

    test_process = ExecuteProcess(
        cmd=[sys.executable, '-c', "print('action')"],
        output='screen'
    )

    test_process.execute(lc)
    assert 'time' not in test_process.process_details['cmd']


def test_execute_process_prefix_filter_override_in_launch_file():
    lc = LaunchContext()
    lc._set_asyncio_loop(osrf_pycommon.process_utils.get_loop())
    SetLaunchConfiguration('launch-prefix', 'time').visit(lc)
    assert len(lc.launch_configurations) == 1
    SetLaunchConfiguration(
        'launch-prefix-filter', 'no-match').visit(lc)
    assert len(lc.launch_configurations) == 2

    test_process = ExecuteProcess(
        prefix='echo',
        cmd=[sys.executable, '-c', "print('action')"],
        output='screen'
    )

    test_process.execute(lc)
    assert 'echo' in test_process.process_details['cmd'] and \
        'time' not in test_process.process_details['cmd']
