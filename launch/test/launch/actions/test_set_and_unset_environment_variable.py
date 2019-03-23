# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the SetEnvironmentVariable and UnsetEnvironmentVariable action classes."""

import os

from launch import LaunchContext
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable
from launch.substitutions import EnvironmentVariable


def test_set_and_unset_environment_variable_constructors():
    """Test the constructor for SetEnvironmentVariable and UnsetEnvironmentVariable classes."""
    SetEnvironmentVariable('name', 'value')
    UnsetEnvironmentVariable('name')


def test_set_and_unset_environment_variable_execute():
    """Test the execute() of the SetEnvironmentVariable and UnsetEnvironmentVariable classes."""
    lc1 = LaunchContext()

    # can set and overwrite environment variables
    if 'NONEXISTANT_KEY' in os.environ:
        del os.environ['NONEXISTANT_KEY']
    assert os.environ.get('NONEXISTANT_KEY') is None
    SetEnvironmentVariable('NONEXISTANT_KEY', 'value').visit(lc1)
    assert os.environ.get('NONEXISTANT_KEY') == 'value'
    SetEnvironmentVariable('NONEXISTANT_KEY', 'ANOTHER_NONEXISTANT_KEY').visit(lc1)
    assert os.environ.get('NONEXISTANT_KEY') == 'ANOTHER_NONEXISTANT_KEY'

    # can unset environment variables
    if 'ANOTHER_NONEXISTANT_KEY' in os.environ:
        del os.environ['ANOTHER_NONEXISTANT_KEY']
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None
    SetEnvironmentVariable('ANOTHER_NONEXISTANT_KEY', 'some value').visit(lc1)
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') == 'some value'
    UnsetEnvironmentVariable('ANOTHER_NONEXISTANT_KEY').visit(lc1)
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None

    # set and unset with substitutions
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None
    SetEnvironmentVariable('ANOTHER_NONEXISTANT_KEY',
                           EnvironmentVariable('NONEXISTANT_KEY')).visit(lc1)
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') == 'ANOTHER_NONEXISTANT_KEY'
    UnsetEnvironmentVariable(EnvironmentVariable('NONEXISTANT_KEY')).visit(lc1)
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None

    # unsetting a environment variable that does not exist doesn't raise an exception
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None
    UnsetEnvironmentVariable(EnvironmentVariable('NONEXISTANT_KEY')).visit(lc1)
    assert os.environ.get('ANOTHER_NONEXISTANT_KEY') is None
