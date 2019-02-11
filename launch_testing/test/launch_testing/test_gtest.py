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

from launch_testing.actions import GTest


def test_gtest():
    """Test running a gtest with timeout."""
    path = Path(__file__).resolve().parents[1] / 'dummy_tests/locking'
    ld = LaunchDescription([
        GTest(
            path=str(path), timeout=5.0,
        )
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
