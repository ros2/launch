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

"""Tests for the boolean substitution classes."""

from launch import LaunchContext

from launch.substitutions import AndSubstitution
from launch.substitutions import NotSubstitution
from launch.substitutions import OrSubstitution
from launch.substitutions.substitution_failure import SubstitutionFailure

import pytest


def test_not_substitution():
    lc = LaunchContext()
    assert NotSubstitution('true').perform(lc) == 'false'
    assert NotSubstitution('false').perform(lc) == 'true'
    assert NotSubstitution('1').perform(lc) == 'false'
    assert NotSubstitution('0').perform(lc) == 'true'
    with pytest.raises(SubstitutionFailure):
        NotSubstitution('not-condition-expression').perform(lc)


def test_and_substitution():
    lc = LaunchContext()
    assert AndSubstitution('true', 'true').perform(lc) == 'true'
    assert AndSubstitution('true', 'false').perform(lc) == 'false'
    assert AndSubstitution('false', 'true').perform(lc) == 'false'
    assert AndSubstitution('false', 'false').perform(lc) == 'false'

    assert AndSubstitution('true', '1').perform(lc) == 'true'
    assert AndSubstitution('true', '0').perform(lc) == 'false'
    assert AndSubstitution('false', '1').perform(lc) == 'false'
    assert AndSubstitution('false', '0').perform(lc) == 'false'

    assert AndSubstitution('1', 'true').perform(lc) == 'true'
    assert AndSubstitution('1', 'false').perform(lc) == 'false'
    assert AndSubstitution('0', 'true').perform(lc) == 'false'
    assert AndSubstitution('0', 'false').perform(lc) == 'false'

    assert AndSubstitution('1', '1').perform(lc) == 'true'
    assert AndSubstitution('1', '0').perform(lc) == 'false'
    assert AndSubstitution('0', '1').perform(lc) == 'false'
    assert AndSubstitution('0', '0').perform(lc) == 'false'

    with pytest.raises(SubstitutionFailure):
        AndSubstitution('not-condition-expression', 'true').perform(lc)
    with pytest.raises(SubstitutionFailure):
        AndSubstitution('true', 'not-condition-expression').perform(lc)


def test_or_substitution():
    lc = LaunchContext()
    assert OrSubstitution('true', 'true').perform(lc) == 'true'
    assert OrSubstitution('true', 'false').perform(lc) == 'true'
    assert OrSubstitution('false', 'true').perform(lc) == 'true'
    assert OrSubstitution('false', 'false').perform(lc) == 'false'

    assert OrSubstitution('true', '1').perform(lc) == 'true'
    assert OrSubstitution('true', '0').perform(lc) == 'true'
    assert OrSubstitution('false', '1').perform(lc) == 'true'
    assert OrSubstitution('false', '0').perform(lc) == 'false'

    assert OrSubstitution('1', 'true').perform(lc) == 'true'
    assert OrSubstitution('1', 'false').perform(lc) == 'true'
    assert OrSubstitution('0', 'true').perform(lc) == 'true'
    assert OrSubstitution('0', 'false').perform(lc) == 'false'

    assert OrSubstitution('1', '1').perform(lc) == 'true'
    assert OrSubstitution('1', '0').perform(lc) == 'true'
    assert OrSubstitution('0', '1').perform(lc) == 'true'
    assert OrSubstitution('0', '0').perform(lc) == 'false'

    with pytest.raises(SubstitutionFailure):
        OrSubstitution('not-condition-expression', 'true').perform(lc)
    with pytest.raises(SubstitutionFailure):
        OrSubstitution('true', 'not-condition-expression').perform(lc)
