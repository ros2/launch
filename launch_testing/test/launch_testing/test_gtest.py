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

"""Tests for the GTest Action."""

from pathlib import Path

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from launch_testing.actions import GTest


def test_gtest():
    """Test running a gtest with timeout."""
    path = Path(__file__).resolve().parents[1] / 'dummy_tests/locking'
    gtest_action = GTest(
                    path=str(path), timeout=5.0,
                )

    def on_gtest_exited(event, context):
        return EmitEvent(event=Shutdown())

    ld = LaunchDescription([
        gtest_action,
        RegisterEventHandler(OnProcessExit(
                on_exit=on_gtest_exited,
                target_action=gtest_action
            ))
    ])
    ls = LaunchService(debug=True)
    ls.include_launch_description(ld)
    assert 0 == ls.run()
