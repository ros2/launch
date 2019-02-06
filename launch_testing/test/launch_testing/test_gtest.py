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

from launch import LaunchDescription
from launch import LaunchService
import subprocess
from launch_testing.actions.gtest import GTest


def test_gtest():
    """Test running a gtest with timeout."""
    subprocess.run(['cd ../../dummy_tests && mkdir build && cd build '
                    '&& cmake .. && make install && cd .. && rm -r build'],
                   shell=True)
    ld = LaunchDescription([
        GTest(
            path='../../dummy_tests/locking', timeout=30.0,
        )
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
