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

"""Tests for the NotEqualsSubstitution class."""

from launch import LaunchContext

from launch.substitutions import NotEqualsSubstitution
from launch.substitutions import PathJoinSubstitution

import os


def test_equals_substitution():
    lc = LaunchContext()
    assert NotEqualsSubstitution(None, None).perform(lc) == 'false'
    assert NotEqualsSubstitution(None, "something").perform(lc) == 'true'
    assert NotEqualsSubstitution(True, True).perform(lc) == 'false'
    assert NotEqualsSubstitution(False, False).perform(lc) == 'false'
    assert NotEqualsSubstitution(True, False).perform(lc) == 'true'
    assert NotEqualsSubstitution(1, True).perform(lc) == 'false'
    assert NotEqualsSubstitution(1, 1).perform(lc) == 'false'
    assert NotEqualsSubstitution(1, 0).perform(lc) == 'true'
    assert NotEqualsSubstitution(1, "1").perform(lc) == 'true'
    assert NotEqualsSubstitution("1", "1").perform(lc) == 'false'

    path = ['asd', 'bsd', 'cds']
    sub = PathJoinSubstitution(path)
    assert NotEqualsSubstitution(sub, os.path.join(*path)).perform(lc) == 'false'
