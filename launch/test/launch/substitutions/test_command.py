# Copyright 2020 Open Source Robotics Foundation, Inc.
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

"""Tests the Command substitution."""

from launch.launch_context import LaunchContext
from launch.substitutions import Command
from launch.substitutions.substitution_failure import SubstitutionFailure

import pytest

def test_command():
    """Test a simple command."""
    context = LaunchContext()
    command = Command('echo asd bsd csd')
    output = command.perform(context)
    assert output == 'asd bsd csd\n'

def test_missing_command_raises():
    """Test that a command that doesn't exist raises."""
    context = LaunchContext()
    command = Command('ros2_launch_test_command_i_m_not_a_command')
    with pytest.raises(SubstitutionFailure) as ex:
        command.perform(context)
    ex.match('File not found:')
