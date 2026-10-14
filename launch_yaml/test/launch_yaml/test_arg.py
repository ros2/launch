# Copyright 2026 Old-Ding
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

"""Test parsing a launch argument."""

import io
import textwrap

from launch.actions import DeclareLaunchArgument

from parser_no_extensions import load_no_extensions

import pytest


def test_arg_choices():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: ur_type
                default: ur5e
                choices: [ur3, ur3e, ur5e]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)

    argument = launch_description.entities[0]
    assert isinstance(argument, DeclareLaunchArgument)
    assert argument.choices == ['ur3', 'ur3e', 'ur5e']


def test_arg_legacy_choices():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: ur_type
                choice:
                    - value: ur3
                    - value: ur3e
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))
    launch_description = parser.parse_description(root_entity)

    argument = launch_description.entities[0]
    assert isinstance(argument, DeclareLaunchArgument)
    assert argument.choices == ['ur3', 'ur3e']


def test_arg_choices_reject_non_string_values():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: ur_type
                choices: [ur3, 5]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))

    with pytest.raises(TypeError, match="'choices'.*list of strings"):
        parser.parse_description(root_entity)


def test_arg_choices_reject_legacy_and_shorthand_together():
    yaml_file = textwrap.dedent(
        """
        launch:
            - arg:
                name: ur_type
                choice:
                    - value: ur3
                choices: [ur3e]
        """
    )
    root_entity, parser = load_no_extensions(io.StringIO(yaml_file))

    with pytest.raises(ValueError, match="Unexpected key.*'choices'"):
        parser.parse_description(root_entity)
