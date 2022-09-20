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

"""Tests for the LaunchLogDir substitution class."""

from launch import LaunchContext
from launch.frontend.parse_substitution import parse_substitution
import launch.logging
from launch.substitutions import LaunchLogDir
from launch.substitutions import TextSubstitution

import pytest


@pytest.fixture
def log_dir(tmpdir_factory):
    """Test fixture that generates a temporary directory for log files."""
    return str(tmpdir_factory.mktemp('logs'))


def test_launch_log_dir():
    """Test the constructors for LaunchLogDir class."""
    LaunchLogDir()


def test_launch_log_dir_methods():
    """Test the methods of the LaunchLogDir class."""
    lld = LaunchLogDir()

    lc = LaunchContext()
    assert lld.perform(lc)


def test_launch_log_dir_frontend(log_dir):
    """Test launch_log_dir/log_dir frontend substitutions."""
    launch.logging.reset()
    launch.logging.launch_config.log_dir = log_dir

    for sub in ('launch_log_dir', 'log_dir'):
        subst = parse_substitution(f'$({sub})')
        assert len(subst) == 1
        result = subst[0]
        assert isinstance(result, TextSubstitution)
        assert result.text == log_dir

        with pytest.raises(TypeError):
            parse_substitution(f'$({sub} some_args)')
