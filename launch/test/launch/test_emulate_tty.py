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

"""Tests for emulate_tty configuration of ExecuteProcess actions."""

import sys

import launch
import pytest


class OnExit(object):
    def __init__(self):
        self.returncode = None

    def handle(self, event, context):
        self.returncode = event.returncode


@pytest.mark.parametrize('test_input,expected', [
    (None, 1),  # use the default defined by ExecuteProcess
    ('true', 1),  # redundantly override the default via LaunchConfiguration
    ('false', 0)  # override the default via LaunchConfiguration
])
def test_emulate_tty(test_input, expected):
    on_exit = OnExit()
    ld = launch.LaunchDescription()
    ld.add_action(launch.actions.ExecuteProcess(
        cmd=[sys.executable, '-c', 'import sys; sys.exit(sys.stdout.isatty())'])
    )
    if test_input is not None:
        ld.add_action(launch.actions.SetLaunchConfiguration('emulate_tty', test_input))
    ld.add_action(
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(on_exit=on_exit.handle)
        )
    )
    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    ls.run()
    assert on_exit.returncode == expected
