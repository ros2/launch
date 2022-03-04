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

"""Tests for the AnonName substitution class."""

from launch import LaunchContext

from launch.substitutions import AndSubstitution
from launch.substitutions import NotSubstitution
from launch.substitutions import OrSubstitution
from launch.substitutions.substitution_failure import SubstitutionFailure

import pytest


@pytest.mark.parametrize('value', ['True', 'true', '1'])
def test_not_substitution_true(value):
    lc = LaunchContext()
    assert NotSubstitution(value).perform(lc) == 'false'


@pytest.mark.parametrize('value', ['False', 'false', '0'])
def test_not_substitution_false(value):
    lc = LaunchContext()
    assert NotSubstitution(value).perform(lc) == 'true'


def test_not_substitution_invalid():
    lc = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        NotSubstitution('not-condition-expression').perform(lc)


@pytest.mark.parametrize('left', ['True', 'true', '1'])
@pytest.mark.parametrize('right', ['True', 'true', '1'])
def test_and_substitution_both_true(left, right):
    lc = LaunchContext()
    assert AndSubstitution(left, right).perform(lc) == 'true'


@pytest.mark.parametrize('left', ['True', 'true', '1'])
@pytest.mark.parametrize('right', ['False', 'false', '0'])
def test_and_substitution_left_true(left, right):
    lc = LaunchContext()
    assert AndSubstitution(left, right).perform(lc) == 'false'


@pytest.mark.parametrize('left', ['False', 'false', '0'])
@pytest.mark.parametrize('right', ['True', 'true', '1'])
def test_and_substitution_right_true(left, right):
    lc = LaunchContext()
    assert AndSubstitution(left, right).perform(lc) == 'false'


@pytest.mark.parametrize('left', ['False', 'false', '0'])
@pytest.mark.parametrize('right', ['False', 'false', '0'])
def test_and_substitution_both_false(left, right):
    lc = LaunchContext()
    assert AndSubstitution(left, right).perform(lc) == 'false'


def test_and_substitution_invalid():
    lc = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        AndSubstitution('not-condition-expression', 'True').perform(lc)
    with pytest.raises(SubstitutionFailure):
        AndSubstitution('True', 'not-condition-expression').perform(lc)


@pytest.mark.parametrize('left', ['True', 'true', '1'])
@pytest.mark.parametrize('right', ['True', 'true', '1'])
def test_or_substitution_both_true(left, right):
    lc = LaunchContext()
    assert OrSubstitution(left, right).perform(lc) == 'true'


@pytest.mark.parametrize('left', ['True', 'true', '1'])
@pytest.mark.parametrize('right', ['False', 'false', '0'])
def test_or_substitution_left_true(left, right):
    lc = LaunchContext()
    assert OrSubstitution(left, right).perform(lc) == 'true'


@pytest.mark.parametrize('left', ['False', 'false', '0'])
@pytest.mark.parametrize('right', ['True', 'true', '1'])
def test_or_substitution_right_true(left, right):
    lc = LaunchContext()
    assert OrSubstitution(left, right).perform(lc) == 'true'


@pytest.mark.parametrize('left', ['False', 'false', '0'])
@pytest.mark.parametrize('right', ['False', 'false', '0'])
def test_or_substitution_both_false(left, right):
    lc = LaunchContext()
    assert OrSubstitution(left, right).perform(lc) == 'false'


def test_or_substitution_invalid():
    lc = LaunchContext()
    with pytest.raises(SubstitutionFailure):
        OrSubstitution('not-condition-expression', 'True').perform(lc)
    with pytest.raises(SubstitutionFailure):
        OrSubstitution('True', 'not-condition-expression').perform(lc)
