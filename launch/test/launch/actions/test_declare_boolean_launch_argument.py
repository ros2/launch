# Copyright 2026 Metro Robots
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

"""Tests for the DeclareBooleanLaunchArgument action class."""

# from launch import LaunchContext
from launch.actions import DeclareBooleanLaunchArgument

import pytest


def test_declare_launch_argument_constructors():
    """Test the constructors for DeclareLaunchArgument class."""
    DeclareBooleanLaunchArgument('name')

    # All possible default values
    for default_value in ['True', 'true', 'TRUE', 'False', 'false', 'FALSE', True, False]:
        DeclareBooleanLaunchArgument('name', default_value=default_value)

    # With description
    DeclareBooleanLaunchArgument('name', description='description')

    # With invalid default value
    with pytest.raises(RuntimeError) as excinfo:
        DeclareBooleanLaunchArgument('name', default_value='invalid')
    assert 'not in provided choices' in str(excinfo.value)

    # With invalid choices override
    with pytest.raises(TypeError) as excinfo:
        DeclareBooleanLaunchArgument('name', choices=['verdad', 'no verdad'])
    assert "multiple values for keyword argument 'choices'" in str(excinfo.value)
