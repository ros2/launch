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

import os

from launch import LaunchContext

from launch.substitutions import EqualsSubstitution
from launch.substitutions import PathJoinSubstitution


def test_equals_substitution():
    def _permute_assertion(left, right, context, output):
        assert EqualsSubstitution(left, right).perform(context) == output
        assert EqualsSubstitution(right, left).perform(context) == output

    lc = LaunchContext()

    # NoneType
    _permute_assertion(None, None, lc, 'true')
    _permute_assertion(None, '', lc, 'true')
    _permute_assertion(None, 'something', lc, 'false')

    # Booleans
    _permute_assertion(True, True, lc, 'true')
    _permute_assertion(False, False, lc, 'true')
    _permute_assertion(True, False, lc, 'false')

    _permute_assertion(True, 'true', lc, 'true')
    _permute_assertion(True, 'True', lc, 'true')
    _permute_assertion(False, 'false', lc, 'true')
    _permute_assertion(False, 'False', lc, 'true')
    _permute_assertion(True, 'False', lc, 'false')

    _permute_assertion(True, 1, lc, 'true')
    _permute_assertion(True, '1', lc, 'true')
    _permute_assertion(True, 0, lc, 'false')
    _permute_assertion(True, '0', lc, 'false')
    _permute_assertion(True, '10', lc, 'false')
    _permute_assertion(True, '-1', lc, 'false')

    _permute_assertion(False, 1, lc, 'false')
    _permute_assertion(False, '1', lc, 'false')
    _permute_assertion(False, 0, lc, 'true')
    _permute_assertion(False, '0', lc, 'true')
    _permute_assertion(False, '10', lc, 'false')
    _permute_assertion(False, '-1', lc, 'false')

    _permute_assertion('true', 1, lc, 'true')
    _permute_assertion('true', '1', lc, 'true')
    _permute_assertion('true', '0', lc, 'false')
    _permute_assertion('true', 'true', lc, 'true')
    _permute_assertion('false', 1, lc, 'false')
    _permute_assertion('false', '1', lc, 'false')
    _permute_assertion('false', '0', lc, 'true')
    _permute_assertion('false', 'false', lc, 'true')
    _permute_assertion('true', 'false', lc, 'false')

    # Numerics
    _permute_assertion(1, 1, lc, 'true')
    _permute_assertion(1, 0, lc, 'false')
    _permute_assertion(1, -1, lc, 'false')
    _permute_assertion(10, 10, lc, 'true')
    _permute_assertion(10, -10, lc, 'false')

    _permute_assertion(10, 10.0, lc, 'true')
    _permute_assertion(10, 10 + 1e-10, lc, 'true')
    _permute_assertion(10, 10.1, lc, 'false')
    _permute_assertion(10.0, -10.0, lc, 'false')

    _permute_assertion(float('nan'), float('nan'), lc, 'false')
    _permute_assertion(float('nan'), 'nan', lc, 'false')
    _permute_assertion('nan', 'nan', lc, 'false')  # Special case

    _permute_assertion(float('inf'), float('inf'), lc, 'true')
    _permute_assertion(float('inf'), 'inf', lc, 'true')
    _permute_assertion('inf', 'inf', lc, 'true')

    _permute_assertion(float('inf'), float('-inf'), lc, 'false')
    _permute_assertion(float('inf'), '-inf', lc, 'false')
    _permute_assertion('inf', '-inf', lc, 'false')
    _permute_assertion('-inf', '-inf', lc, 'true')

    # Strings
    _permute_assertion('wow', 'wow', lc, 'true')
    _permute_assertion('wow', True, lc, 'false')
    _permute_assertion('wow', 1, lc, 'false')
    _permute_assertion('wow', 0, lc, 'false')
    _permute_assertion('wow', 10, lc, 'false')

    # Substitutions
    path = ['asd', 'bsd', 'cds']
    sub = PathJoinSubstitution(path)
    assert EqualsSubstitution(sub, os.path.join(*path)).perform(lc) == 'true'
