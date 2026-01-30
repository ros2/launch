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

"""Test parsing a DeclareLaunchArgument action with choices."""

import io
import textwrap

from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.frontend import Parser
from launch.launch_context import LaunchContext

import pytest


def test_declare_launch_argument_choice():
    """Test declaring a launch argument with choices via YAML."""
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: mode
                default: 'A'
                choice:
                    - value: 'A'
                    - value: 'B'
        """
    )
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)

    assert len(ld.entities) == 1
    assert isinstance(ld.entities[0], DeclareLaunchArgument)
    assert ld.entities[0].name == 'mode'
    assert ld.entities[0].choices == ['A', 'B']

    # Test with valid choice (default)
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
    assert ls.context.launch_configurations['mode'] == 'A'


def test_declare_launch_argument_invalid_choice():
    """Test declaring a launch argument with an invalid choice via YAML."""
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: mode
                default: 'C'
                choice:
                    - value: 'A'
                    - value: 'B'
        """
    )
    root_entity, parser = Parser.load(io.StringIO(yaml_file))
    ld = parser.parse_description(root_entity)

    lc = LaunchContext()
    with pytest.raises(RuntimeError) as excinfo:
        ld.entities[0].visit(lc)
    assert "is not valid. Valid options are: ['A', 'B']" in str(excinfo.value)


if __name__ == '__main__':
    test_declare_launch_argument_choice()
    test_declare_launch_argument_invalid_choice()
