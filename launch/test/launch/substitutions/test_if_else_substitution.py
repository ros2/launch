# Copyright 2023 Open Source Robotics Foundation, Inc.
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

"""Tests for the IfElseSubstitution substitution class."""

from launch import LaunchContext
from launch.substitutions import IfElseSubstitution

import pytest


def test_if_else_substitution_no_values():
    """Check that construction fails if no values are specified."""
    # Should raise an error since neither the if value nor the else value is given
    with pytest.raises(RuntimeError):
        IfElseSubstitution('true')


def test_if_else_substitution_both_values():
    """Check that the right value is returned when both values are given."""
    # Condition is true case
    subst = IfElseSubstitution('true', 'ivalue', 'evalue')
    result = subst.perform(LaunchContext())
    assert result == 'ivalue'
    subst = IfElseSubstitution('true', if_value='ivalue', else_value='evalue')
    result = subst.perform(LaunchContext())
    assert result == 'ivalue'

    # Condition is false case
    subst = IfElseSubstitution('false', 'ivalue', 'evalue')
    result = subst.perform(LaunchContext())
    assert result == 'evalue'


def test_if_else_substitution_if_value():
    """Check that the right value is returned when only the if value is given."""
    # Condition is true case
    subst = IfElseSubstitution('1', 'ivalue')
    result = subst.perform(LaunchContext())
    assert result == 'ivalue'
    subst = IfElseSubstitution('1', if_value='ivalue')
    result = subst.perform(LaunchContext())
    assert result == 'ivalue'

    # Condition is false case
    subst = IfElseSubstitution('0', 'ivalue')
    result = subst.perform(LaunchContext())
    assert result == ''


def test_if_else_substitution_else_value():
    """Check that the right value is returned when only the else value is given."""
    # Condition is true case
    subst = IfElseSubstitution('on', else_value='evalue')
    result = subst.perform(LaunchContext())
    assert result == ''

    # Condition is false case
    subst = IfElseSubstitution('off', else_value='evalue')
    result = subst.perform(LaunchContext())
    assert result == 'evalue'
