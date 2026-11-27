# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Tests for the NotEmptySubstitution substitution class."""

from launch import LaunchContext
from launch.substitutions import NotEmptySubstitution
from launch.substitutions import TextSubstitution

import pytest


@pytest.mark.parametrize('value, expected', [
    ('', 'false'),
    ('robot', 'true'),
    (' ', 'true'),
    ('\t\n\r', 'true'),
])
def test_not_empty(value, expected):
    """Test checking if a string is not empty."""
    substitution = NotEmptySubstitution(value)
    assert substitution.perform(LaunchContext()) == expected


def test_not_empty_nested_substitutions():
    """Test checking if a value assembled from multiple substitutions is not empty."""
    substitution = NotEmptySubstitution([
        TextSubstitution(text='robot'),
        TextSubstitution(text=' name'),
    ])
    assert substitution.perform(LaunchContext()) == 'true'


def test_not_empty_parse():
    """Test the frontend parser contract."""
    substitution_type, kwargs = NotEmptySubstitution.parse(['some value'])
    assert substitution_type is NotEmptySubstitution
    assert kwargs == {'value': 'some value'}

    with pytest.raises(TypeError, match='expects 1 argument'):
        NotEmptySubstitution.parse([])
    with pytest.raises(TypeError, match='expects 1 argument'):
        NotEmptySubstitution.parse(['one', 'two'])
