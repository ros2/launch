# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Tests for the StringJoinSubstitution substitution class."""

from launch import LaunchContext
from launch.substitutions import StringJoinSubstitution
from launch.substitutions import TextSubstitution


def test_string_join():
    context = LaunchContext()

    strings = ['abc', 'def', 'ghi']
    sub = StringJoinSubstitution(strings)
    assert sub.perform(context) == ''.join(strings)

    strings = ['abc', ['def'], [TextSubstitution(text='ghi'), 'jkl'], TextSubstitution(text='mno')]
    sub_with_sub = StringJoinSubstitution(strings)
    assert sub_with_sub.perform(context) == 'abcdefghijklmno'


def test_string_join_with_delimiter():
    context = LaunchContext()

    strings = ['abc', 'def', 'ghi']
    sub = StringJoinSubstitution(strings, delimiter='.')
    assert sub.perform(context) == '.'.join(strings)

    strings = ['abc', ['def'], [TextSubstitution(text='ghi'), 'jkl'], TextSubstitution(text='mno')]
    sub_with_sub = StringJoinSubstitution(strings, delimiter='.')
    assert sub_with_sub.perform(context) == 'abc.def.ghijkl.mno'
