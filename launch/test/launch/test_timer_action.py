# Copyright 2019 Apex.AI, Inc.
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


"""Tests for the TimerAction Action."""
import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import NotSubstitution
from launch.substitutions import PythonExpression


def test_timer_action_can_capture_the_environment():
    """Test that timer actions capture the environment variables present when executed."""
    # Regression test for https://github.com/ros2/launch_ros/issues/376
    shutdown_reasons = []

    variable_is_set = PythonExpression([
        "'",
        EnvironmentVariable('LAUNCH_TEST_ENV_VAR', default_value='BAR'),
        "' == 'FOO'"
    ])

    ld = LaunchDescription([

        ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        GroupAction(
            actions=[

                SetEnvironmentVariable(
                    name='LAUNCH_TEST_ENV_VAR',
                    value='FOO'
                ),

                TimerAction(
                    period=1.,
                    actions=[

                        Shutdown(
                            condition=IfCondition(variable_is_set),
                            reason='Test passing'
                        ),

                        Shutdown(
                            condition=IfCondition(NotSubstitution(variable_is_set)),
                            reason='Test failing'
                        ),
                    ]
                ),
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 1
    assert shutdown_reasons[0].reason == 'Test passing'


def test_multiple_launch_with_timers():
    # Regression test for https://github.com/ros2/launch/issues/183
    # Unfortunately, when things aren't working this test just hangs on the second call to
    # ls.run

    def generate_launch_description():
        return LaunchDescription([

            ExecuteProcess(
                cmd=[sys.executable, '-c', 'while True: pass'],
            ),

            TimerAction(
                period=1.,
                actions=[
                    Shutdown(reason='Timer expired')
                ]
            )
        ])

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    assert 0 == ls.run()  # Always works

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    # Next line hangs forever before https://github.com/ros2/launch/issues/183 was fixed.
    assert 0 == ls.run()


def _shutdown_listener_factory(reasons_arr):
    return RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: reasons_arr.append(event)
        )
    )


def test_timer_action_sanity_check():
    """Test that timer actions work (sanity check)."""
    # This test is structured like test_shutdown_preempts_timers and
    # test_timer_can_block_preemption as a sanity check that the shutdown listener
    # and other launch related infrastructure works as expected
    shutdown_reasons = []

    ld = LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        TimerAction(
            period=1.,
            actions=[
                Shutdown(reason='One second timeout')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert shutdown_reasons[0].reason == 'One second timeout'


def test_shutdown_preempts_timers():
    shutdown_reasons = []

    ld = LaunchDescription([

        ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        TimerAction(
            period=1.,
            actions=[
                Shutdown(reason='fast shutdown')
            ]
        ),

        TimerAction(
            period=2.,
            actions=[
                Shutdown(reason='slow shutdown')
            ]
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 1
    assert shutdown_reasons[0].reason == 'fast shutdown'


def test_timer_can_block_preemption():
    shutdown_reasons = []

    ld = LaunchDescription([

        ExecuteProcess(
            cmd=[sys.executable, '-c', 'while True: pass'],
        ),

        TimerAction(
            period=1.,
            actions=[
                Shutdown(reason='fast shutdown')
            ]
        ),

        TimerAction(
            period=2.,
            actions=[
                Shutdown(reason='slow shutdown')
            ],
            cancel_on_shutdown=False  # Preempted in test_shutdown_preempts_timers, but not here
        ),

        _shutdown_listener_factory(shutdown_reasons),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert len(shutdown_reasons) == 2  # Should see 'shutdown' event twice because
    assert shutdown_reasons[0].reason == 'fast shutdown'
    assert shutdown_reasons[1].reason == 'slow shutdown'


def test_timer_action_launch_configurations():
    # The timer action's entities should have access to the launch configurations at the time the
    # timer action executed
    ld = LaunchDescription([
        GroupAction(
            # Causes the launch configurations to be reset after the timer action executes, which
            # would cause 'launch_arg' to not exist
            scoped=True,
            actions=[
                SetLaunchConfiguration('launch_arg', 'launch_arg_value'),
                TimerAction(
                    period=1.0,
                    actions=[
                        LogInfo(
                            msg=LaunchConfiguration('launch_arg'),
                        ),
                    ],
                ),
            ],
        ),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    # However, we do not want the timer action's entities to affect the context, e.g., leak launch
    # configurations out of the GroupAction in this case
    assert 'launch_arg' not in ls.context.launch_configurations
