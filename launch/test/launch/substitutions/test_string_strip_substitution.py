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

"""Tests for the StringStripSubstitution substitution class."""

from launch import LaunchContext
from launch.substitutions import StringStripSubstitution
from launch.substitutions import TextSubstitution

import pytest


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        ('', ''),
        ('robot', 'robot'),
        (' \trobot\r\n', 'robot'),
        (' \t\r\n', ''),
    ],
)
def test_string_strip(value, expected):
    """Test stripping leading and trailing whitespace."""
    substitution = StringStripSubstitution(value)
    assert substitution.perform(LaunchContext()) == expected


def test_string_strip_nested_substitutions():
    """Test stripping a value assembled from multiple substitutions."""
    substitution = StringStripSubstitution([
        ' \t',
        TextSubstitution(text='robot'),
        TextSubstitution(text=' name'),
        '\r\n',
    ])
    assert substitution.perform(LaunchContext()) == 'robot name'


def test_string_strip_parse():
    """Test the frontend parser contract."""
    substitution_type, kwargs = StringStripSubstitution.parse([' value '])
    assert substitution_type is StringStripSubstitution
    assert kwargs == {'value': ' value '}

    with pytest.raises(TypeError, match='expects 1 argument'):
        StringStripSubstitution.parse([])
    with pytest.raises(TypeError, match='expects 1 argument'):
        StringStripSubstitution.parse(['one', 'two'])
