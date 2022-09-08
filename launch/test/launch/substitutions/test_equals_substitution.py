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

"""Tests for the EqualsSubstitution class."""

from launch import LaunchContext

from launch.substitutions import EqualsSubstitution
from launch.substitutions import PathJoinSubstitution

import os


def test_equals_substitution():
    lc = LaunchContext()
    assert EqualsSubstitution(None, None).perform(lc) == 'true'
    assert EqualsSubstitution(None, "something").perform(lc) == 'false'
    assert EqualsSubstitution(True, True).perform(lc) == 'true'
    assert EqualsSubstitution(False, False).perform(lc) == 'true'
    assert EqualsSubstitution(True, False).perform(lc) == 'false'
    assert EqualsSubstitution(1, True).perform(lc) == 'true'
    assert EqualsSubstitution(1, 1).perform(lc) == 'true'
    assert EqualsSubstitution(1, 0).perform(lc) == 'false'
    assert EqualsSubstitution(1, "1").perform(lc) == 'false'
    assert EqualsSubstitution("1", "1").perform(lc) == 'true'

    path = ['asd', 'bsd', 'cds']
    sub = PathJoinSubstitution(path)
    assert EqualsSubstitution(sub, os.path.join(*path)).perform(lc) == 'true'
