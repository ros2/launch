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

"""Tests for the FindLaunchfile substitution class."""

from pathlib import Path

from launch import LaunchContext
from launch.frontend import Parser
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import FindLaunchfile
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions
import pytest

TEST_DIR = Path(__file__).parent / 'test_find_launchfile'
# Fake some valid extensions, since those packages aren't available here
Parser.frontend_parsers = {  # type: ignore
    'yaml': None,
    'xml': None,
}


def test_fullname():
    assert FindLaunchfile(name='a_launch.py', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='a.launch.xml', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='b_launch.yaml', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='c.py', path=TEST_DIR).perform(LaunchContext())


def test_valid_suffix():
    assert FindLaunchfile(name='a_launch', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='a.launch', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='b', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='b_launch', path=TEST_DIR).perform(LaunchContext())
    assert FindLaunchfile(name='c', path=TEST_DIR).perform(LaunchContext())


def test_invalid_suffix():
    with pytest.raises(SubstitutionFailure):
        FindLaunchfile(name='b_l', path=TEST_DIR).perform(LaunchContext())


def test_notfound():
    with pytest.raises(SubstitutionFailure):
        FindLaunchfile(name='d', path=TEST_DIR).perform(LaunchContext())


def test_multiple():
    with pytest.raises(SubstitutionFailure):
        FindLaunchfile(name='a', path=TEST_DIR).perform(LaunchContext())


def test_frontend():
    subst = parse_substitution('$(find-launchfile foo bar)')
    assert len(subst) == 1
    result = subst[0]
    assert isinstance(result, FindLaunchfile)
    assert perform_substitutions(LaunchContext(), result.name) == 'foo'
    assert perform_substitutions(LaunchContext(), result.path) == 'bar'
