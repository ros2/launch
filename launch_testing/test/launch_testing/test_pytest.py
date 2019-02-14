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

"""Tests for the PyTest Action."""

from pathlib import Path

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_testing.actions import PyTest


def run_dummy_test(file):
    """Utilitie to run both test files."""
    path = Path(__file__).resolve().parents[1] / 'dummy_tests' / file
    pytest_action = PyTest(
        path=str(path), timeout=5.0,
    )

    def on_pytest_exited(event, context):
        return EmitEvent(event=Shutdown())

    ld = LaunchDescription([
        pytest_action,
        RegisterEventHandler(OnProcessExit(
                on_exit=on_pytest_exited,
                target_action=pytest_action
            ))
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()


def test_pytest_locking():
    """Test running a locking pytest with timeout."""
    run_dummy_test('locking.py')


def test_pytest_non_locking():
    """Test running a non-locking pytest with timeout."""
    run_dummy_test('dummy.py')
