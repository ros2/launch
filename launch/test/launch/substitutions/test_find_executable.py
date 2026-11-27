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

"""Tests for the FindExecutable substitution."""

import sys

from launch import LaunchContext
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import SubstitutionFailure

import pytest


@pytest.mark.parametrize('use_launch_configuration', [False, True])
def test_missing_executable_reports_resolved_name(monkeypatch, tmp_path, use_launch_configuration):
    """Report the evaluated name rather than the internal substitution objects."""
    monkeypatch.setenv('PATH', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations['tool'] = 'missing_launch_test_tool'
    name = [LaunchConfiguration('tool'), '_suffix'] if use_launch_configuration \
        else 'missing_launch_test_tool_suffix'

    with pytest.raises(SubstitutionFailure) as exc_info:
        FindExecutable(name=name).perform(context)

    assert str(exc_info.value) == (
        "executable 'missing_launch_test_tool_suffix' not found on the PATH")


def test_find_existing_executable():
    """Return the path of an executable after resolving a launch configuration."""
    context = LaunchContext()
    context.launch_configurations['tool'] = sys.executable

    assert FindExecutable(name=LaunchConfiguration('tool')).perform(context) == sys.executable
